import bpy

import re
import itertools
import bisect
import math

from rigify.utils.errors import MetarigError
from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.bones import put_bone
from rigify.utils.mechanism import make_driver, make_constraint, driver_var_transform
from rigify.utils.widgets import create_widget
from rigify.utils.widgets_basic import create_circle_widget, create_sphere_widget
from rigify.utils.layers import ControlLayersOption
from rigify.utils.misc import map_list, map_apply
from rigify.utils.animation import add_generic_snap_fk_to_ik
from rigify.utils.switch_parent import SwitchParentBuilder

from rigify.base_rig import stage

from rigify.rigs.chain_rigs import SimpleChainRig
from rigify.rigs.widgets import create_gear_widget

from itertools import count, repeat


class Rig(SimpleChainRig):
    ##############################
    # Initialization

    def initialize(self):
        super(Rig,self).initialize()

        org_chain = self.bones.org
        org_bones = [self.get_bone(org) for org in org_chain]

        # Compute master bone name: inherit .LR suffix, but strip trailing digits
        name_parts = re.match(r'^(.*?)(?:([._-])\d+)?((?:[._-][LlRr])?)(?:\.\d+)?$', strip_org(org_chain[0]))
        name_base, name_sep, name_suffix = name_parts.groups()

        self.name_base = name_base
        self.name_sep = name_sep if name_sep else '-'
        self.name_suffix = name_suffix

        # Find the spline object if it exists
        self.spline_name = self.obj.name + '-MCH-' + name_base + name_sep + 'spline' + name_suffix
        self.spline_obj = None

        if self.generator.id_store.rigify_generate_mode == 'overwrite':
            if self.spline_name in bpy.data.objects:
                self.spline_obj = bpy.data.objects[self.spline_name]

                if not isinstance(self.spline_obj.data, bpy.types.Curve):
                    raise MetarigError("Object '%s' already exists and is not a curve." % (self.spline_name))

                if self.spline_obj.parent and self.spline_obj.parent != self.obj:
                    raise MetarigError("Object '%s' already exists and is not a child of the rig." % (self.spline_name))

        # Options
        self.use_stretch = (self.params.sik_stretch_control == 'MANUAL_STRETCH')
        self.use_tip = (self.params.sik_stretch_control == 'DIRECT_TIP')
        self.use_fk = self.params.sik_fk_controls

        # Compute org chain lengths and control distribution
        if self.use_tip:
            org_bones.pop()

        self.org_lengths = [bone.length for bone in org_bones]
        self.org_totlengths = list(itertools.accumulate(self.org_lengths))
        self.chain_length = self.org_totlengths[-1]
        self.avg_length = self.chain_length / len(org_bones)

        end_idx = len(org_bones) - 1

        # Find which bones hold main controls
        self.num_main_controls = self.params.sik_mid_controls + 2
        main_control_step = self.chain_length / (self.num_main_controls - 1)

        self.main_control_poslist = [
            self.find_bone_by_length(i * main_control_step) + (self.get_main_control_name(i),)
            for i in range(self.num_main_controls)
        ]

        # Likewise for extra start and end controls
        num_start_controls = self.params.sik_start_controls
        start_range = main_control_step / self.org_lengths[0]
        start_control_step = start_range * 0.001 / max(1, num_start_controls)

        self.start_control_poslist = [
            (0, (i + 1) * start_control_step, self.make_name('start%02d' % (idx+1)))
            for idx, i in enumerate(reversed(range(num_start_controls)))
        ]

        num_end_controls = self.params.sik_end_controls + (1 if self.use_tip else 0)
        end_range = main_control_step / self.org_lengths[-1]
        end_control_step = end_range * 0.001 / max(1, num_end_controls)

        self.end_control_poslist = [
            (end_idx, 1.0 - (i + 1) * end_control_step, self.make_name('end%02d' % (idx+1)))
            for idx, i in enumerate(reversed(range(num_end_controls)))
        ]

        # Adjust control bindings if using manual tip control
        if self.use_tip:
            tip = self.main_control_poslist[-1]
            self.main_control_poslist[-1] = (end_idx+1, 0, strip_org(org_chain[-1]))

            tip_extra = self.end_control_poslist[0]
            self.end_control_poslist[0] = (end_idx, max(0, 1 - end_range * 0.25), self.make_name('end'))

        # Radius scaling
        self.use_radius = self.params.sik_radius_scaling
        self.max_curve_radius = self.params.sik_max_radius if self.use_radius else 1.0


    ##############################
    # Utilities

    def find_bone_by_length(self, pos):
        totlengths = self.org_totlengths
        idx = bisect.bisect_left(totlengths, pos)
        idx = min(idx, len(totlengths) - 1)
        prev = totlengths[idx - 1] if idx > 0 else 0
        return (idx, min(1.0, (pos - prev) / (totlengths[idx] - prev)))

    def make_name(self, midpart):
        "Make a name for a bone not tied to a specific org bone"
        return self.name_base + self.name_sep + midpart + self.name_suffix

    def get_main_control_name(self, i):
        if i == 0:
            base = 'start'
        elif i == self.num_main_controls - 1:
            base = 'end'
        else:
            base = 'mid%02d' % (i)

        return self.make_name(base)

    def make_bone_by_spec(self, pos_spec, name, scale):
        "Make a bone positioned along the chain."
        org_name = self.bones.org[pos_spec[0]]
        new_name = self.copy_bone(org_name, name, parent=False)

        org_bone = self.get_bone(org_name)
        new_bone = self.get_bone(new_name)
        new_bone.translate(pos_spec[1] * (org_bone.tail - org_bone.head))
        new_bone.length = self.avg_length * scale

        return new_name

    ENABLE_CONTROL_PROPERTY = [None, 'start_controls', 'end_controls']

    def rig_enable_control_driver(self, owner, prop, subtype, index, disable=False):
        if subtype != 0:
            if self.use_tip and subtype == 2:
                if index == 0:
                    return
                index -= 1

            master = self.bones.ctrl.master
            varprop = self.ENABLE_CONTROL_PROPERTY[subtype]

            make_driver(
                owner, prop,
                expression='active %s %d' % ('<=' if disable else '>', index),
                variables={'active':(self.obj, master, varprop)}
            )

    ##############################
    # BONES
    #
    # ctrl:
    #   master:
    #     Root control for moving and scaling the whole rig.
    #   main[]:
    #     List of main spline controls (always visible and active).
    #   start[], end[]:
    #     List of extra spline controls attached to the tip main ones (can disable).
    #   end_twist:
    #     Twist control at the end of the tentacle.
    #   fk[]:
    #     FK control chain.
    # mch:
    #   start_parent, end_parent:
    #     Intermediate bones for parenting extra controls - discards scale of the main.
    #   start_hooks[], end_hooks[]:
    #     Proxy bones for extra control hooks.
    #   ik[]:
    #     Spline IK chain, responsible for extracting the shape of the curve.
    #   ik_final[]:
    #     Final IK result with tip_fix.
    #   end_stretch:
    #     Bone used in distributing the end twist control scaling over the chain.
    #   tip_fix_parent, tip_fix
    #     Bones used to match tip control rotation and scale.
    #
    ##############################

    ##############################
    # Master control bone

    @stage.generate_bones
    def make_master_control_bone(self):
        self.bones.ctrl.master = self.copy_bone(
            self.bones.org[0], self.make_name('master'), parent=True,
            length = self.avg_length * 1.5
        )

        self.register_parents()

    def register_parents(self):
        builder = SwitchParentBuilder(self.generator)

        builder.register_parent(self, self.bones.ctrl.master)

        if self.use_tip:
            builder.register_parent(self, self.bones.org[-1], exclude_self=True)

    @stage.configure_bones
    def configure_master_control_bone(self):
        master = self.bones.ctrl.master
        ctrls = self.bones.ctrl.flatten()
        rig_name = self.name_base + self.name_suffix

        # Properties for enabling extra controls
        if self.params.sik_start_controls > 0:
            self.make_property(
                master, 'start_controls', 0,
                min=0, max=self.params.sik_start_controls,
                description="Enabled extra start controls for "+rig_name
            )

            panel = self.script.panel_with_selected_check(ctrls)
            panel.custom_prop(master, 'start_controls', text="Start Controls")

        if self.params.sik_end_controls > 0:
            self.make_property(
                master, 'end_controls', 0,
                min=0, max=self.params.sik_end_controls,
                description="Enabled extra end controls for "+rig_name
            )

            panel = self.script.panel_with_selected_check(ctrls)
            panel.custom_prop(master, 'end_controls', text="End Controls")

        if self.use_tip:
            maxval = len(self.bones.org) / 2

            self.make_property(
                master, 'end_twist', 0.0, min=-maxval, max=maxval,
                description="Rough end twist estimate in full rotations. The rig auto-corrects it to the actual tip orientation within 180 degrees"
            )

            panel = self.script.panel_with_selected_check(ctrls)
            panel.custom_prop(master, 'end_twist', text="End Twist Fix")

        if self.use_fk:
            self.make_property(master, 'IK_FK', 0.0, description='IK/FK switch for '+rig_name)

            panel = self.script.panel_with_selected_check(ctrls)
            panel.custom_prop(master, 'IK_FK', text="IK - FK", slider=True)

            ik_controls = [ item[0] for item in self.all_controls ]
            if not self.use_tip:
                ik_controls += [ self.bones.ctrl.end_twist ]

            add_generic_snap_fk_to_ik(
                self.generator, panel_controls=ctrls,
                fk_bones=self.bones.ctrl.fk, ik_bones=self.get_ik_final(), ik_ctrl_bones=ik_controls,
                undo_copy_scale=True,
                extra_text=' (%s)' % (rig_name)
            )

    @stage.generate_widgets
    def make_master_control_widget(self):
        master_name = self.bones.ctrl.master
        create_gear_widget(self.obj, master_name, size=5.0)

    ##############################
    # Twist controls

    @stage.generate_bones
    def make_twist_control_bones(self):
        if not self.use_tip:
            self.bones.ctrl.end_twist = self.make_twist_control_bone('end-twist', 1.15)

    def make_twist_control_bone(self, name, size):
        return self.copy_bone(self.bones.org[0], self.make_name(name), length=self.avg_length * size)

    @stage.parent_bones
    def parent_twist_control_bones(self):
        if not self.use_tip:
            self.set_bone_parent(self.bones.ctrl.end_twist, self.bones.ctrl.master)

    @stage.configure_bones
    def configure_twist_control_bones(self):
        if not self.use_tip:
            self.configure_twist_control_bone(self.bones.ctrl.end_twist)

    def configure_twist_control_bone(self, name):
        bone = self.get_bone(name)
        bone.rotation_mode = 'XYZ'
        bone.lock_location = (True, True, True)
        bone.lock_rotation = (True, False, True)
        if not self.use_stretch:
            bone.lock_scale = (True, True, True)

    @stage.rig_bones
    def rig_twist_control_bones(self):
        if not self.use_tip:
            # Copy the location of the end bone to provide more convenient tool behavior.
            self.make_constraint(self.bones.ctrl.end_twist, 'COPY_LOCATION', self.bones.org[-1])

    @stage.generate_widgets
    def make_twist_control_widgets(self):
        if not self.use_tip:
            self.make_twist_control_widget(self.bones.ctrl.end_twist, self.bones.org[-1], 0.75)

    def make_twist_control_widget(self, ctrl, org, size=1.0, head_tail=0.5):
        bone = self.get_bone(ctrl)
        bone_org = self.get_bone(org)

        scale = bone_org.length / bone.length

        bone.custom_shape_transform = bone_org
        bone.custom_shape_scale = scale

        create_twist_widget(self.obj, ctrl, size=size/scale, head_tail=head_tail)

    ##############################
    # Twist controls MCH

    @stage.generate_bones
    def make_mch_twist_control_bones(self):
        if self.use_stretch:
            self.bones.mch.end_stretch = self.make_mch_end_stretch_bone('end-twist.stretch', 1.15)

    def make_mch_end_stretch_bone(self, name_base, size):
        name = make_derived_name(self.make_name(name_base), 'mch')
        return self.copy_bone(self.bones.org[0], name, length = self.avg_length * size * 0.5)

    @stage.parent_bones
    def parent_mch_twist_control_bones(self):
        if self.use_stretch:
            self.set_bone_parent(self.bones.mch.end_stretch, self.bones.ctrl.master)

    @stage.rig_bones
    def rig_mch_twist_control_bones(self):
        if self.use_stretch:
            self.rig_mch_end_stretch_bone(self.bones.mch.end_stretch, self.bones.ctrl.end_twist)

    def rig_mch_end_stretch_bone(self, mch, ctrl):
        # Break the dependency cycle caused by COPY_LOCATION above by copying raw properties.
        self.make_driver(mch, 'scale', index=0, variables=[(ctrl, '.scale.x')])
        self.make_driver(mch, 'scale', index=1, variables=[(ctrl, '.scale.y')])
        self.make_driver(mch, 'scale', index=2, variables=[(ctrl, '.scale.z')])

        self.make_constraint(mch, 'MAINTAIN_VOLUME', mode='UNIFORM', owner_space='LOCAL')

    ##############################
    # Spline controls

    @stage.generate_bones
    def make_main_control_chain(self):
        self.bones.ctrl.main = map_list(self.make_main_control_bone, self.main_control_poslist)
        self.bones.ctrl.start = map_list(self.make_extra_control_bone, self.start_control_poslist)
        self.bones.ctrl.end = map_list(self.make_extra_control_bone, self.end_control_poslist)

        self.make_all_controls_list()
        self.make_controls_switch_parent()

    def make_all_controls_list(self):
        main_controls = [(bone, 0, i) for i, bone in enumerate(self.bones.ctrl.main)]
        start_controls = [(bone, 1, i) for i, bone in enumerate(self.bones.ctrl.start)]
        end_controls = [(bone, 2, i) for i, bone in enumerate(self.bones.ctrl.end)]

        self.tip_controls_table = [None, self.bones.ctrl.main[0], self.bones.ctrl.main[-1]]
        self.all_controls = [main_controls[0], *reversed(start_controls), *main_controls[1:-1], *end_controls, main_controls[-1]]

    def make_controls_switch_parent(self):
        builder = SwitchParentBuilder(self.generator)

        extra = lambda: [
            (self.bones.mch.start_parent, self.bones.ctrl.main[0]),
            (self.bones.mch.end_parent, self.bones.ctrl.main[-1])
        ]
        select_table = [
            lambda: self.bones.ctrl.master,
            lambda: self.bones.mch.start_parent,
            lambda: self.bones.mch.end_parent
        ]

        for (bone, subtype, index) in self.all_controls[1:-1]:
            builder.build_child(
                self, bone, extra_parents=extra,
                select_parent=select_table[subtype],
                no_fix_rotation=True, no_fix_scale=True
            )

        builder.build_child(self, self.bones.ctrl.main[-1], no_fix_scale=not self.use_tip)

    def make_main_control_bone(self, pos_spec):
        return self.make_bone_by_spec(pos_spec, pos_spec[2], 1.1)

    def make_extra_control_bone(self, pos_spec):
        return self.make_bone_by_spec(pos_spec, pos_spec[2], 0.9)

    @stage.parent_bones
    def parent_main_control_chain(self):
        self.set_bone_parent(self.bones.ctrl.main[0], self.bones.ctrl.master)

    @stage.configure_bones
    def configure_main_control_chain(self):
        for info in self.all_controls:
            self.configure_main_control_bone(*info)

    def configure_main_control_bone(self, ctrl, subtype, index):
        bone = self.get_bone(ctrl)

        can_rotate = False

        if subtype == 0 and index == 0:
            if self.params.sik_start_controls > 0:
                bone.rotation_mode = 'QUATERNION'
            else:
                bone.rotation_mode = 'XYZ'
                bone.lock_rotation = (True, False, True)
        elif (subtype == 0 and index == self.num_main_controls-1
              and self.params.sik_end_controls > 0):
            bone.rotation_mode = 'QUATERNION'
        else:
            bone.lock_rotation_w = True
            bone.lock_rotation = (True, True, True)

        if subtype == 0 and index == 0:
            bone.lock_scale = (False, not self.use_stretch, False)
        elif not self.use_radius:
            bone.lock_scale = (True, True, True)

    @stage.rig_bones
    def rig_main_control_chain(self):
        for info in self.all_controls:
            self.rig_main_control_bone(*info)

    def rig_main_control_bone(self, ctrl, subtype, index):
        if self.use_stretch and subtype == 0 and index == 0:
            self.make_constraint(ctrl, 'MAINTAIN_VOLUME', mode='UNIFORM', owner_space='LOCAL')

        self.rig_enable_control_driver(self.get_bone(ctrl).bone, 'hide', subtype, index, disable=True)

    @stage.generate_widgets
    def make_main_control_widgets(self):
        for info in self.all_controls:
            self.make_main_control_widget(*info)

    def make_main_control_widget(self, ctrl, subtype, index):
        if subtype == 0 and index == 0:
            if len(self.start_control_poslist) > 0:
                create_twist_widget(self.obj, ctrl, size=1, head_tail=0.25)
            else:
                self.make_twist_control_widget(ctrl, self.bones.org[0], head_tail=0.25)
        elif self.use_tip and subtype == 0 and index == self.num_main_controls - 1:
            create_circle_widget(self.obj, ctrl, radius=0.5, head_tail=0.25)
        else:
            create_sphere_widget(self.obj, ctrl)

    ##############################
    # FK Control chain

    @stage.generate_bones
    def make_control_chain(self):
        if self.use_fk:
            super().make_control_chain()

    @stage.parent_bones
    def parent_control_chain(self):
        if self.use_fk:
            super().parent_control_chain()

            self.set_bone_parent(self.bones.ctrl.fk[0], self.bones.ctrl.master)

    @stage.configure_bones
    def configure_control_chain(self):
        if self.use_fk:
            super().configure_control_chain()

            ControlLayersOption.FK.assign(self.params, self.obj, self.bones.ctrl.fk)

    @stage.rig_bones
    def rig_control_chain(self):
        if self.use_fk:
            for args in zip(self.bones.ctrl.fk, [None] + self.bones.ctrl.fk):
                self.rig_control_bone(*args)

    def rig_control_bone(self, fk, fk_prev):
        if fk_prev:
            self.get_bone(fk).bone.use_inherit_scale = False
            self.make_constraint(fk, 'COPY_SCALE', fk_prev, use_offset=True, space='POSE')

    @stage.generate_widgets
    def make_control_widgets(self):
        if self.use_fk:
            super().make_control_widgets()

    def make_control_widget(self, ctrl):
        create_circle_widget(self.obj, ctrl, radius=0.3, head_tail=0.5)

    ##############################
    # Spline tip parent MCH

    @stage.generate_bones
    def make_mch_extra_parent_bones(self):
        self.bones.mch.start_parent = self.make_mch_extra_parent_bone(self.main_control_poslist[0])
        self.bones.mch.end_parent = self.make_mch_extra_parent_bone(self.main_control_poslist[-1])

    def make_mch_extra_parent_bone(self, pos_spec):
        return self.make_bone_by_spec(pos_spec, make_derived_name(pos_spec[2], 'mch', '.psocket'), 0.40)

    @stage.parent_bones
    def parent_mch_extra_parent_bones(self):
        self.set_bone_parent(self.bones.mch.start_parent, self.bones.ctrl.master)
        self.set_bone_parent(self.bones.mch.end_parent, self.bones.ctrl.master)

    @stage.rig_bones
    def rig_mch_extra_parent_bones(self):
        self.rig_mch_extra_parent_bone(self.bones.mch.start_parent, self.bones.ctrl.main[0])
        self.rig_mch_extra_parent_bone(self.bones.mch.end_parent, self.bones.ctrl.main[-1])

    def rig_mch_extra_parent_bone(self, bone, ctrl):
        self.make_constraint(bone, 'COPY_LOCATION', ctrl)
        self.make_constraint(bone, 'COPY_ROTATION', ctrl)

    ##############################
    # Spline extra hook proxy MCH

    @stage.generate_bones
    def make_mch_extra_hook_bones(self):
        self.bones.mch.start_hooks = map_list(self.make_mch_extra_hook_bone, self.start_control_poslist)
        self.bones.mch.end_hooks = map_list(self.make_mch_extra_hook_bone, self.end_control_poslist)

        self.mch_hooks_table = [ None, self.bones.mch.start_hooks, self.bones.mch.end_hooks ]

    def make_mch_extra_hook_bone(self, pos_spec):
        return self.make_bone_by_spec(pos_spec, make_derived_name(pos_spec[2], 'mch', '.hook'), 0.30)

    @stage.parent_bones
    def parent_mch_extra_hook_bones(self):
        for hook in self.bones.mch.start_hooks:
            self.set_bone_parent(hook, self.bones.mch.start_parent)

        for hook in self.bones.mch.end_hooks:
            self.set_bone_parent(hook, self.bones.mch.end_parent)

    @stage.rig_bones
    def rig_mch_extra_hook_bones(self):
        for (bone, subtype, index) in self.all_controls:
            hooks = self.mch_hooks_table[subtype]
            if hooks:
                self.rig_mch_extra_hook_bone(hooks[index], bone, subtype, index)

    def rig_mch_extra_hook_bone(self, hook, ctrl, subtype, index):
        tip_ctrl = self.tip_controls_table[subtype]

        con = self.make_constraint(hook, 'COPY_LOCATION', ctrl)
        self.rig_enable_control_driver(con, 'mute', subtype, index, disable=True)

        con = self.make_constraint(hook, 'COPY_SCALE', ctrl, space='LOCAL')
        self.rig_enable_control_driver(con, 'mute', subtype, index, disable=True)

        if subtype == 2 and not self.use_tip:
            con = self.make_constraint(hook, 'COPY_SCALE', tip_ctrl, space='LOCAL')
            self.rig_enable_control_driver(con, 'mute', subtype, index, disable=False)

    ##############################
    # Spline Object

    @stage.configure_bones
    def make_spline_object(self):
        if not self.spline_obj:
            spline_data = bpy.data.curves.new(self.spline_name, 'CURVE')
            self.spline_obj = bpy.data.objects.new(self.spline_name, spline_data)
            self.generator.collection.objects.link(self.spline_obj)

            self.spline_obj.show_in_front = True
            self.spline_obj.hide_select = True
            self.spline_obj.hide_render = True
            #self.spline_obj.hide_viewport = True

        self.spline_obj.animation_data_clear()
        self.spline_obj.data.animation_data_clear()

        self.spline_obj.shape_key_clear()
        self.spline_obj.modifiers.clear()

        spline_data = self.spline_obj.data

        spline_data.splines.clear()
        spline_data.dimensions = '3D'

        self.make_spline_points(spline_data, self.all_controls)

        if self.use_radius:
            self.make_spline_keys(self.spline_obj, self.all_controls)

        self.spline_obj.parent = self.obj
        self.spline_obj.parent_type = 'OBJECT'

    def make_spline_points(self, spline_data, all_controls):
        spline = spline_data.splines.new('BEZIER')

        spline.bezier_points.add(len(all_controls) - 1)

        end_index = len(all_controls) - 1

        for i, (name,subtype,index) in enumerate(all_controls):
            point = spline.bezier_points[i]
            point.handle_left_type = point.handle_right_type = 'AUTO'
            point.co = point.handle_left = point.handle_right = self.get_bone(name).head

            if i == 0 or self.use_tip and i == end_index:
                point.radius = 1.0
            else:
                point.radius = self.max_curve_radius

    def make_spline_keys(self, spline_obj, all_controls):
        spline_obj.shape_key_add(name='Basis', from_mix=False)

        controls = all_controls[1:-1] if self.use_tip else all_controls[1:]

        for i, (name,subtype,index) in enumerate(controls):
            key = spline_obj.shape_key_add(name=name, from_mix=False)
            key.value = 0.0
            key.data[i+1].radius = 0.0


    @stage.rig_bones
    def rig_spline_object(self):
        for i, info in enumerate(self.all_controls):
            self.rig_spline_hook(i, *info)

        if self.use_radius:
            controls = self.all_controls[1:-1] if self.use_tip else self.all_controls[1:]

            for i, info in enumerate(controls):
                self.rig_spline_radius_shapekey(i, *info)

    def rig_spline_hook(self, i, ctrl, subtype, index):
        hooks = self.mch_hooks_table[subtype]
        bone = self.get_bone(ctrl)

        hook = self.spline_obj.modifiers.new(ctrl, 'HOOK')
        hook.object = self.obj
        hook.subtarget = hooks[index] if hooks else ctrl
        hook.center = bone.head
        hook.vertex_indices_set([i*3, i*3 + 1, i*3 + 2])

    def rig_spline_radius_shapekey(self, i, ctrl, subtype, index):
        key = self.spline_obj.data.shape_keys.key_blocks[i + 1]
        switch_prop = self.ENABLE_CONTROL_PROPERTY[subtype]

        assert key.name == ctrl

        hooks = self.mch_hooks_table[subtype]
        target = hooks[index] if hooks else ctrl

        expr = '1 - var / %.2f' % (self.max_curve_radius)
        scale_var = [driver_var_transform(self.obj, target, type='SCALE_AVG', space='LOCAL')]

        make_driver(key, 'value', expression=expr, variables=scale_var)

    ##############################
    # Spline IK Chain MCH

    @stage.generate_bones
    def make_mch_ik_chain(self):
        orgs = self.bones.org[0:-1] if self.use_tip else self.bones.org
        self.bones.mch.ik = map_list(self.make_mch_ik_bone, orgs)

    def make_mch_ik_bone(self, org):
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.ik'))
        self.get_bone(name).use_inherit_scale = False
        return name

    @stage.parent_bones
    def parent_mch_ik_chain(self):
        self.parent_bone_chain(self.bones.mch.ik, use_connect=True)
        self.set_bone_parent(self.bones.mch.ik[0], self.bones.ctrl.main[0])

    @stage.rig_bones
    def rig_mch_ik_chain(self):
        for i, args in enumerate(zip(self.bones.mch.ik)):
            self.rig_mch_ik_bone(i, *args)

        self.rig_mch_ik_constraint(self.bones.mch.ik[-1])

    def rig_mch_ik_bone(self, i, mch):
        self.get_bone(mch).rotation_mode = 'XYZ'

        num_ik = len(self.bones.org)

        # Apply end twist rotation
        if self.use_tip:
            rot_fac = math.pi * 2 / num_ik
            rot_var = [(self.bones.ctrl.master, 'end_twist')]
        else:
            rot_fac = 1.0 / num_ik
            rot_var = [(self.bones.ctrl.end_twist, '.rotation_euler.y')]

        self.make_driver(mch, 'rotation_euler', index=1, expression='var * %f' % (rot_fac), variables=rot_var)

        # Copy the common scale
        self.make_constraint(mch, 'COPY_SCALE', self.bones.ctrl.main[0])

        if self.use_stretch:
            self.make_constraint(
                mch, 'COPY_SCALE', self.bones.mch.end_stretch,
                use_offset = True, space = 'LOCAL',
                power = (i + 1) / num_ik
            )

    def rig_mch_ik_constraint(self, mch):
        ik_bone = self.get_bone(mch)

        make_constraint(
            ik_bone, 'SPLINE_IK', self.spline_obj,
            chain_count = len(self.bones.mch.ik),
            use_curve_radius = self.use_radius,
            y_scale_mode = 'BONE_ORIGINAL' if self.use_stretch else 'FIT_CURVE',
            xz_scale_mode = 'VOLUME_PRESERVE',
            use_original_scale = True,
        )

    ##############################
    # Tip matching MCH

    @stage.generate_bones
    def make_mch_tip_fix(self):
        if self.use_tip:
            org = self.bones.org[-1]
            parent = self.copy_bone(org, make_derived_name(org, 'mch', '.fix.parent'), scale=0.8)
            self.get_bone(parent).use_inherit_scale = False
            self.bones.mch.tip_fix_parent = parent
            self.bones.mch.tip_fix = self.copy_bone(org, make_derived_name(org, 'mch', '.fix'), scale=0.7)

    @stage.parent_bones
    def parent_mch_tip_fix(self):
        if self.use_tip:
            self.set_bone_parent(self.bones.mch.tip_fix_parent, self.bones.mch.ik[-1])
            self.set_bone_parent(self.bones.mch.tip_fix, self.bones.mch.tip_fix_parent)

    @stage.rig_bones
    def rig_mch_tip_fix(self):
        if self.use_tip:
            ctrl = self.bones.ctrl.main[-1]
            parent = self.bones.mch.tip_fix_parent
            fix = self.bones.mch.tip_fix

            # Rig the baseline bone as the end of the IK chain (scale, twist)
            self.rig_mch_ik_bone(len(self.bones.mch.ik), parent)

            # Align the baseline to the tip control direction
            self.make_constraint(parent, 'DAMPED_TRACK', ctrl, head_tail=1.0)

            # Deduce the scale and twist correction by subtracting baseline
            # from tip control transform via parenting and local space.
            self.make_constraint(fix, 'COPY_TRANSFORMS', ctrl)

    ###################################
    # Final IK Chain MCH (tip matched)

    @stage.generate_bones
    def make_mch_ik_final_chain(self):
        if self.use_tip:
            self.bones.mch.ik_final = map_list(self.make_mch_ik_final_bone, self.bones.org[0:-1])

    def make_mch_ik_final_bone(self, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '.ik.final'))

    def get_ik_final(self):
        if self.use_tip:
            return [*self.bones.mch.ik_final, self.bones.ctrl.main[-1] ]
        else:
            return self.bones.mch.ik

    @stage.parent_bones
    def parent_mch_ik_final_chain(self):
        if self.use_tip:
            for final, ik in zip(self.bones.mch.ik_final, self.bones.mch.ik):
                self.set_bone_parent(final, ik)

    @stage.rig_bones
    def rig_mch_ik_final_chain(self):
        if self.use_tip:
            for args in zip(count(0), self.bones.mch.ik_final):
                self.rig_mch_ik_final_bone(*args)

    def rig_mch_ik_final_bone(self, i, mch):
        fix = self.bones.mch.tip_fix
        factor = (i + 1) / len(self.bones.org)

        self.make_constraint(
            mch, 'COPY_ROTATION', fix, space='LOCAL',
            use_x=False, use_z=False, influence=factor
        )
        self.make_constraint(
            mch, 'COPY_SCALE', fix, space='LOCAL',
            use_y=False, power=factor
        )

    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org[0], self.bones.ctrl.master)

    @stage.rig_bones
    def rig_org_chain(self):
        for args in zip(count(0), self.bones.org, self.get_ik_final()):
            self.rig_org_bone(*args)

    def rig_org_bone(self, i, org, ik):
        self.make_constraint(org, 'COPY_TRANSFORMS', ik)

        if self.use_fk:
            con = self.make_constraint(org, 'COPY_TRANSFORMS', self.bones.ctrl.fk[i])

            self.make_driver(con, 'influence', variables=[(self.bones.ctrl.master, 'IK_FK')])

    ##############################
    # UI

    @classmethod
    def add_parameters(self, params):
        """ Register the rig parameters. """

        params.sik_start_controls = bpy.props.IntProperty(
            name="Extra Start Controls", min=0, default=1,
            description="Number of extra spline control points attached to the start control"
        )
        params.sik_mid_controls = bpy.props.IntProperty(
            name="Middle Controls", min=1, default=1,
            description="Number of spline control points in the middle"
        )
        params.sik_end_controls = bpy.props.IntProperty(
            name="Extra End Controls", min=0, default=1,
            description="Number of extra spline control points attached to the end control"
        )

        params.sik_stretch_control = bpy.props.EnumProperty(
            name="Tip Control",
            description="How the stretching of the tentacle is controlled",
            items=[('FIT_CURVE', 'Stretch To Fit', 'The tentacle stretches to fit the curve'),
                   ('DIRECT_TIP', 'Direct Tip Control',
                    'The last bone of the chain is directly controlled, like the hand in an IK arm, '+
                    'and the middle stretches to reach it'),
                   ('MANUAL_STRETCH', 'Manual Squash & Stretch',
                    'The tentacle scaling is manually controlled via twist controls.')]
        )

        params.sik_radius_scaling = bpy.props.BoolProperty(
            name="Radius Scaling", default=True,
            description="Allow scaling the spline control bones to affect the thickness via curve radius"
        )
        params.sik_max_radius = bpy.props.FloatProperty(
            name="Maximum Radius", min=1, default=10,
            description="Maximum supported scale factor for the spline control bones"
        )

        params.sik_fk_controls = bpy.props.BoolProperty(
            name="FK Controls", default=True,
            description="Generate an FK control chain for the tentacle"
        )

        ControlLayersOption.FK.add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        """ Create the ui for the rig parameters. """

        layout.label(icon='INFO', text='A straight line rest shape works best.')

        layout.prop(params, 'sik_start_controls')
        layout.prop(params, 'sik_mid_controls')
        layout.prop(params, 'sik_end_controls')

        layout.prop(params, 'sik_stretch_control', text='')

        layout.prop(params, 'sik_radius_scaling')

        col = layout.column()
        col.active = params.sik_radius_scaling
        col.prop(params, 'sik_max_radius')

        layout.prop(params, 'sik_fk_controls')

        col = layout.column()
        col.active = params.sik_fk_controls
        ControlLayersOption.FK.parameters_ui(col, params)


def create_twist_widget(rig, bone_name, size=1.0, head_tail=0.5, bone_transform_name=None):
    obj = create_widget(rig, bone_name, bone_transform_name)
    if obj != None:
        verts = [(0.3429814279079437*size, head_tail, 0.22917263209819794*size),
                 (0.38110050559043884*size, head_tail-0.05291016772389412*size, 0.15785686671733856*size),
                 (0.40457412600517273*size, head_tail-0.05291016772389412*size, 0.08047471195459366*size),
                 (0.41250014305114746*size, head_tail-0.05291016772389412*size, -2.671097298900804e-08*size),
                 (0.40457412600517273*size, head_tail-0.05291016772389412*size, -0.08047476410865784*size),
                 (0.38110050559043884*size, head_tail-0.05291016772389412*size, -0.15785691142082214*size),
                 (0.3429814279079437*size, head_tail, -0.22917278110980988*size),
                 (0.22917293012142181*size, head_tail, -0.3429813086986542*size),
                 (0.1578570008277893*size, head_tail-0.05291016772389412*size, -0.3811003565788269*size),
                 (0.0804748609662056*size, head_tail-0.05291016772389412*size, -0.4045739769935608*size),
                 (5.8895523125102045e-08*size, head_tail-0.05291026830673218*size, -0.4124999940395355*size),
                 (-0.08047471195459366*size, head_tail-0.05291016772389412*size, -0.4045739769935608*size),
                 (-0.15785688161849976*size, head_tail-0.05291016772389412*size, -0.38110026717185974*size),
                 (-0.22917267680168152*size, head_tail, -0.3429811894893646*size),
                 (-0.34298115968704224*size, head_tail, -0.22917254269123077*size),
                 (-0.38110023736953735*size, head_tail-0.05291016772389412*size, -0.15785665810108185*size),
                 (-0.4045737385749817*size, head_tail-0.05291016772389412*size, -0.08047446608543396*size),
                 (-0.4124998152256012*size, head_tail-0.05291016772389412*size, 3.4045575603158795e-07*size),
                 (-0.40457355976104736*size, head_tail-0.05291016772389412*size, 0.08047513663768768*size),
                 (-0.3810998201370239*size, head_tail-0.05291016772389412*size, 0.1578572690486908*size),
                 (-0.34298068284988403*size, head_tail, 0.22917301952838898*size),
                 (-0.2291719913482666*size, head_tail, 0.34298139810562134*size),
                 (-0.15785618126392365*size, head_tail-0.05291016772389412*size, 0.38110047578811646*size),
                 (-0.08047392964363098*size, head_tail-0.05291016772389412*size, 0.40457388758659363*size),
                 (8.555148269806523e-07*size, head_tail-0.05291016772389412*size, 0.41249993443489075*size),
                 (0.08047562092542648*size, head_tail-0.05291016772389412*size, 0.4045736789703369*size),
                 (0.15785779058933258*size, head_tail-0.05291016772389412*size, 0.3810998797416687*size),
                 (0.22917351126670837*size, head_tail, 0.3429807126522064*size),
                 (0.38110050559043884*size, head_tail+0.05290994420647621*size, 0.15785686671733856*size),
                 (0.40457412600517273*size, head_tail+0.05290994420647621*size, 0.08047470450401306*size),
                 (0.41250014305114746*size, head_tail+0.05290994420647621*size, -3.1336515604607484e-08*size),
                 (0.40457412600517273*size, head_tail+0.05290994420647621*size, -0.08047477155923843*size),
                 (0.38110050559043884*size, head_tail+0.05290994420647621*size, -0.15785691142082214*size),
                 (0.1578570008277893*size, head_tail+0.05290994420647621*size, -0.3811003565788269*size),
                 (0.0804748609662056*size, head_tail+0.05290994420647621*size, -0.4045739769935608*size),
                 (5.8895523125102045e-08*size, head_tail+0.05290984362363815*size, -0.4124999940395355*size),
                 (-0.08047471195459366*size, head_tail+0.05290994420647621*size, -0.4045739769935608*size),
                 (-0.15785688161849976*size, head_tail+0.05290994420647621*size, -0.38110026717185974*size),
                 (-0.38110023736953735*size, head_tail+0.05290994420647621*size, -0.15785665810108185*size),
                 (-0.4045737385749817*size, head_tail+0.05290994420647621*size, -0.08047447353601456*size),
                 (-0.4124998152256012*size, head_tail+0.05290994420647621*size, 3.35830208086918e-07*size),
                 (-0.40457355976104736*size, head_tail+0.05290994420647621*size, 0.08047512918710709*size),
                 (-0.3810998201370239*size, head_tail+0.05290994420647621*size, 0.1578572690486908*size),
                 (-0.15785618126392365*size, head_tail+0.05290994420647621*size, 0.38110047578811646*size),
                 (-0.08047392964363098*size, head_tail+0.05290994420647621*size, 0.40457388758659363*size),
                 (8.555148269806523e-07*size, head_tail+0.05290994420647621*size, 0.41249993443489075*size),
                 (0.08047562092542648*size, head_tail+0.05290994420647621*size, 0.4045736789703369*size),
                 (0.15785779058933258*size, head_tail+0.05290994420647621*size, 0.3810998797416687*size), ]
        edges = [(1, 0), (2, 1), (2, 3), (3, 4), (5, 4), (5, 6), (7, 8), (9, 8), (10, 9), (10, 11),
                 (12, 11), (12, 13), (14, 15), (16, 15), (16, 17), (17, 18), (19, 18), (20, 19),
                 (28, 0), (21, 22), (23, 22), (23, 24), (24, 25), (26, 25), (26, 27), (47, 27),
                 (29, 28), (29, 30), (30, 31), (32, 31), (32, 6), (34, 33), (35, 34), (35, 36),
                 (37, 36), (7, 33), (37, 13), (39, 38), (39, 40), (40, 41), (42, 41), (14, 38),
                 (20, 42), (44, 43), (44, 45), (45, 46), (47, 46), (21, 43),]
        faces = []

        mesh = obj.data
        mesh.from_pydata(verts, edges, faces)
        mesh.update()
        mesh.update()
        return obj
    else:
        return None


def create_sample(obj):
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('base')
    bone.head[:] = 0.0000, 0.0000, 0.0000
    bone.tail[:] = 0.0000, 0.0000, 0.3000
    bone.roll = 0.0000
    bone.use_connect = False
    bones['base'] = bone.name
    bone = arm.edit_bones.new('tentacle01')
    bone.head[:] = 0.0000, 0.0000, 0.3000
    bone.tail[:] = 0.0000, 0.0000, 0.4400
    bone.roll = 0.0000
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['base']]
    bones['tentacle01'] = bone.name
    bone = arm.edit_bones.new('tentacle02')
    bone.head[:] = 0.0000, 0.0000, 0.4400
    bone.tail[:] = 0.0000, 0.0000, 0.5800
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['tentacle01']]
    bones['tentacle02'] = bone.name
    bone = arm.edit_bones.new('tentacle03')
    bone.head[:] = 0.0000, 0.0000, 0.5800
    bone.tail[:] = 0.0000, 0.0000, 0.7200
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['tentacle02']]
    bones['tentacle03'] = bone.name
    bone = arm.edit_bones.new('tentacle04')
    bone.head[:] = 0.0000, 0.0000, 0.7200
    bone.tail[:] = 0.0000, 0.0000, 0.8600
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['tentacle03']]
    bones['tentacle04'] = bone.name
    bone = arm.edit_bones.new('tentacle05')
    bone.head[:] = 0.0000, 0.0000, 0.8600
    bone.tail[:] = 0.0000, 0.0000, 1.0000
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['tentacle04']]
    bones['tentacle05'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['base']]
    pbone.rigify_type = 'basic.super_copy'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['tentacle01']]
    pbone.rigify_type = 'limbs.spline_tentacle'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['tentacle02']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['tentacle03']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['tentacle04']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['tentacle05']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'

    bpy.ops.object.mode_set(mode='EDIT')
    for bone in arm.edit_bones:
        bone.select = False
        bone.select_head = False
        bone.select_tail = False
    for b in bones:
        bone = arm.edit_bones[bones[b]]
        bone.select = True
        bone.select_head = True
        bone.select_tail = True
        arm.edit_bones.active = bone
