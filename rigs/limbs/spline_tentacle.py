import bpy

import re
import itertools
import bisect

from rigify.utils.errors import MetarigError
from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.bones import put_bone
from rigify.utils.mechanism import make_driver, make_constraint, driver_var_transform
from rigify.utils.widgets import create_widget
from rigify.utils.widgets_basic import create_circle_widget, create_sphere_widget
from rigify.utils.layers import ControlLayersOption
from rigify.utils.misc import map_list, map_apply

from rigify.base_rig import *

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
        name_parts = re.match(r'^(.*?)(?:([._-])?\d+)?((?:[._-][LlRr])?)(?:\.\d+)?$', strip_org(org_chain[0]))
        name_base, name_sep, name_suffix = name_parts.groups()
        name_base += name_sep if name_sep else '-'

        self.name_base = name_base
        self.name_suffix = name_suffix

        # Find the spline object if it exists
        self.spline_name = self.obj.name + '-MCH-' + name_base + 'spline' + name_suffix
        self.spline_obj = None

        if self.generator.id_store.rigify_generate_mode == 'overwrite':
            if self.spline_name in bpy.data.objects:
                self.spline_obj = bpy.data.objects[self.spline_name]

                if not isinstance(self.spline_obj.data, bpy.types.Curve):
                    raise MetarigError("Object '%s' already exists and is not a curve." % (self.spline_name))

                if self.spline_obj.parent and self.spline_obj.parent != self.obj:
                    raise MetarigError("Object '%s' already exists and is not a child of the rig." % (self.spline_name))

        # Compute org chain lengths and control distribution
        self.org_lengths = [bone.length for bone in org_bones]
        self.org_totlengths = list(itertools.accumulate(self.org_lengths))
        self.chain_length = self.org_totlengths[-1]
        self.avg_length = self.chain_length / len(org_chain)

        # Find which bones hold main controls
        self.num_main_controls = self.params.sik_mid_controls + 2
        main_control_step = self.chain_length / (self.num_main_controls - 1)

        self.main_control_poslist = [self.find_bone_by_length(i * main_control_step)
                                     for i in range(self.num_main_controls)]

        # Likewise for extra start and end controls
        num_start_controls = self.params.sik_start_controls
        start_control_step = main_control_step * 0.001 / self.org_lengths[0] / max(1, num_start_controls)

        self.start_control_poslist = [(0, (i + 1) * start_control_step)
                                      for i in reversed(range(num_start_controls))]

        num_end_controls = self.params.sik_end_controls
        end_control_step = main_control_step * 0.001 / self.org_lengths[-1] / max(1, num_end_controls)

        self.end_control_poslist = [(len(org_bones) - 1, 1.0 - (i + 1) * end_control_step)
                                    for i in reversed(range(num_end_controls))]

        # Radius scaling
        self.use_radius = self.params.sik_radius_scaling
        self.max_curve_radius = self.params.sik_max_radius if self.use_radius else 1.0

        self.use_stretch = self.params.sik_stretch_control


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
        return self.name_base + midpart + self.name_suffix

    def get_main_control_name(self, i):
        if i == 0:
            return 'start'
        elif i == self.num_main_controls - 1:
            return 'end'
        else:
            return 'mid%02d' % (i)

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
    #   main:
    #     List of main spline controls (always visible and active).
    #   start, end:
    #     List of extra spline controls attached to the tip main ones (can disable).
    #   start_twist, end_twist:
    #     Twist controls at the ends of the tentacle.
    # mch:
    #   start_parent, end_parent
    #     Intermediate bones for parenting extra controls - discards scale of the main.
    #   ik:
    #     Spline IK chain, responsible for extracting the shape of the curve.
    #   end_stretch:
    #     Bone used in distributing the end twist control scaling over the chain.
    #
    ##############################

    ##############################
    # Master control bone

    @stage_generate_bones
    def make_master_control_bone(self):
        name = self.copy_bone(self.bones.org[0], self.make_name('master'), parent=True)
        self.bones.ctrl.master = name
        self.get_bone(name).length = self.avg_length * 1.5

    @stage_configure_bones
    def configure_master_control_bone(self):
        master = self.bones.ctrl.master
        ctrls = self.bones.ctrl.flatten()

        # Properties for enabling extra controls
        if self.params.sik_start_controls > 0:
            self.make_property(
                master, 'start_controls', 0,
                min=0, max=self.params.sik_start_controls,
                description="Enabled extra start controls"
            )

            self.script.add_panel_selected_check(ctrls)
            self.script.add_panel_custom_prop(master, 'start_controls', text="Start Controls")

        if self.params.sik_end_controls > 0:
            self.make_property(
                master, 'end_controls', 0,
                min=0, max=self.params.sik_end_controls,
                description="Enabled extra end controls"
            )

            self.script.add_panel_selected_check(ctrls)
            self.script.add_panel_custom_prop(master, 'end_controls', text="End Controls")

    @stage_generate_widgets
    def make_master_control_widget(self):
        master_name = self.bones.ctrl.master
        create_gear_widget(self.obj, master_name, size=5.0)

    ##############################
    # Twist controls

    @stage_generate_bones
    def make_twist_control_bones(self):
        self.bones.ctrl.start_twist = self.make_twist_control_bone('start-twist', 1.35)
        self.bones.ctrl.end_twist = self.make_twist_control_bone('end-twist', 1.15)

        if self.use_stretch:
            self.bones.mch.end_stretch = self.make_mch_end_stretch_bone(self.bones.ctrl.end_twist)

    def make_twist_control_bone(self, name, size):
        name = self.copy_bone(self.bones.org[0], self.make_name(name))
        self.get_bone(name).length = self.avg_length * size
        return name

    def make_mch_end_stretch_bone(self, end_ctrl):
        name = self.copy_bone(end_ctrl, make_derived_name(end_ctrl, 'mch', '.stretch'))
        self.get_bone(name).length *= 0.5
        return name

    @stage_parent_bones
    def parent_twist_control_bones(self):
        self.set_bone_parent(self.bones.ctrl.start_twist, self.bones.ctrl.master)
        self.set_bone_parent(self.bones.ctrl.end_twist, self.bones.ctrl.master)

        if self.use_stretch:
            self.set_bone_parent(self.bones.mch.end_stretch, self.bones.ctrl.master)

    @stage_configure_bones
    def configure_twist_control_bones(self):
        self.configure_twist_control_bone(self.bones.ctrl.start_twist)
        self.configure_twist_control_bone(self.bones.ctrl.end_twist)

    def configure_twist_control_bone(self, name):
        bone = self.get_bone(name)
        bone.rotation_mode = 'XYZ'
        bone.lock_location = (True, True, True)
        bone.lock_rotation = (True, False, True)
        if not self.use_stretch:
            bone.lock_scale = (True, True, True)

    @stage_rig_bones
    def rig_twist_control_bones(self):
        # Copy the location of the end bone to provide more convenient tool behavior.
        self.make_constraint(self.bones.ctrl.end_twist, 'COPY_LOCATION', self.bones.org[-1])

        if self.use_stretch:
            self.make_constraint(self.bones.ctrl.start_twist, 'MAINTAIN_VOLUME', mode='UNIFORM', owner_space='LOCAL')

            self.rig_mch_end_stretch_bone(self.bones.mch.end_stretch, self.bones.ctrl.end_twist)

    def rig_mch_end_stretch_bone(self, mch, ctrl):
        expr = 'pow(max(1e-5, var0), 1/%d)' % (len(self.bones.mch.ik))

        self.make_driver(mch, 'scale', index=0, expression=expr, variables=[(ctrl, '.scale.x')])
        self.make_driver(mch, 'scale', index=1, expression=expr, variables=[(ctrl, '.scale.y')])
        self.make_driver(mch, 'scale', index=2, expression=expr, variables=[(ctrl, '.scale.z')])

        self.make_constraint(mch, 'MAINTAIN_VOLUME', mode='UNIFORM', owner_space='LOCAL')

    @stage_generate_widgets
    def make_twist_control_widgets(self):
        self.make_twist_control_widget(self.bones.ctrl.start_twist, self.bones.org[0], False)
        self.make_twist_control_widget(self.bones.ctrl.end_twist, self.bones.org[-1], True)

    def make_twist_control_widget(self, ctrl, org, is_end):
        bone = self.get_bone(ctrl)
        bone_org = self.get_bone(org)

        pos = 0.75 if is_end else 0.25
        scale = bone_org.length / bone.length

        bone.custom_shape_transform = bone_org
        bone.custom_shape_scale = scale

        create_twist_widget(self.obj, ctrl, size=1/scale, head_tail=pos)

    ##############################
    # Spline controls

    @stage_generate_bones
    def make_control_chain(self):
        self.bones.ctrl.main = map_list(self.make_main_control_bone, self.main_control_poslist, count(0))
        self.bones.ctrl.start = map_list(self.make_extra_control_bone, self.start_control_poslist, count(0), repeat('start'))
        self.bones.ctrl.end = map_list(self.make_extra_control_bone, self.end_control_poslist, count(0), repeat('end'))

        self.make_all_controls_list()

    def make_all_controls_list(self):
        main_controls = [(bone, 0, i) for i, bone in enumerate(self.bones.ctrl.main)]
        start_controls = [(bone, 1, i) for i, bone in enumerate(self.bones.ctrl.start)]
        end_controls = [(bone, 2, i) for i, bone in enumerate(self.bones.ctrl.end)]

        self.tip_controls = [None, self.bones.ctrl.main[0], self.bones.ctrl.main[-1]]
        self.all_controls = [main_controls[0], *reversed(start_controls), *main_controls[1:-1], *end_controls, main_controls[-1]]

    def make_main_control_bone(self, pos_spec, i):
        name = self.get_main_control_name(i)
        return self.make_bone_by_spec(pos_spec, self.make_name(name), 0.80)

    def make_extra_control_bone(self, pos_spec, i, namebase):
        return self.make_bone_by_spec(pos_spec, self.make_name('%s%02d' % (namebase, i+1)), 0.70)

    @stage_parent_bones
    def parent_control_chain(self):
        main_bones = self.bones.ctrl.main
        map_apply(self.parent_main_control_bone, main_bones)
        map_apply(self.parent_extra_control_bone, self.bones.ctrl.start, repeat(self.bones.mch.start_parent))
        map_apply(self.parent_extra_control_bone, self.bones.ctrl.end, repeat(self.bones.mch.end_parent))

    def parent_main_control_bone(self, ctrl):
        self.set_bone_parent(ctrl, self.bones.ctrl.master)

    def parent_extra_control_bone(self, ctrl, base):
        self.set_bone_parent(ctrl, base)

    @stage_configure_bones
    def configure_control_chain(self):
        for info in self.all_controls:
            self.configure_control_bone(*info)

    def configure_control_bone(self, ctrl, subtype, index):
        bone = self.get_bone(ctrl)

        can_rotate = False

        if subtype == 0:
            if index == 0 and self.params.sik_start_controls > 0:
                can_rotate = True
            elif index == self.num_main_controls-1 and self.params.sik_end_controls > 0:
                can_rotate = True

        if can_rotate:
            bone.rotation_mode = 'QUATERNION'
        else:
            bone.lock_rotation_w = True
            bone.lock_rotation = (True, True, True)

        if not self.use_radius:
            bone.lock_scale = (True, True, True)

    @stage_rig_bones
    def rig_control_chain(self):
        for info in self.all_controls:
            self.rig_control_bone(*info)

    def rig_control_bone(self, ctrl, subtype, index):
        self.rig_enable_control_driver(self.get_bone(ctrl).bone, 'hide', subtype, index, disable=True)

    @stage_generate_widgets
    def make_control_widgets(self):
        for info in self.all_controls:
            self.make_control_widget(*info)

    def make_control_widget(self, ctrl, subtype, index):
        create_sphere_widget(self.obj, ctrl)

    ##############################
    # Spline tip parent MCH

    # IN-STAGE DEPENDS ON make_control_chain
    @stage_generate_bones
    def make_mch_extra_parent_bones(self):
        if len(self.start_control_poslist) > 0:
            self.bones.mch.start_parent = self.make_mch_extra_parent_bone(self.bones.ctrl.main[0])

        if len(self.end_control_poslist) > 0:
            self.bones.mch.end_parent = self.make_mch_extra_parent_bone(self.bones.ctrl.main[-1])

    def make_mch_extra_parent_bone(self, base_bone):
        name = self.copy_bone(base_bone, make_derived_name(base_bone, 'mch', '.psocket'))
        self.get_bone(name).length *= 0.5
        return name

    @stage_parent_bones
    def parent_mch_extra_parent_bones(self):
        if len(self.start_control_poslist) > 0:
            self.set_bone_parent(self.bones.mch.start_parent, self.bones.ctrl.master)

        if len(self.end_control_poslist) > 0:
            self.set_bone_parent(self.bones.mch.end_parent, self.bones.ctrl.master)

    @stage_rig_bones
    def rig_mch_extra_parent_bones(self):
        if len(self.start_control_poslist) > 0:
            self.rig_mch_extra_parent_bone(self.bones.mch.start_parent, self.bones.ctrl.main[0])

        if len(self.end_control_poslist) > 0:
            self.rig_mch_extra_parent_bone(self.bones.mch.end_parent, self.bones.ctrl.main[-1])

    def rig_mch_extra_parent_bone(self, bone, ctrl):
        self.make_constraint(bone, 'COPY_LOCATION', ctrl)
        self.make_constraint(bone, 'COPY_ROTATION', ctrl)

    ##############################
    # Spline Object

    @stage_configure_bones
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

        for i, (name,subtype,index) in enumerate(all_controls):
            point = spline.bezier_points[i]
            point.handle_left_type = point.handle_right_type = 'AUTO'
            point.co = point.handle_left = point.handle_right = self.get_bone(name).head
            point.radius = self.max_curve_radius

    def make_spline_keys(self, spline_obj, all_controls):
        spline_obj.shape_key_add(name='Basis', from_mix=False)

        for i, (name,subtype,index) in enumerate(all_controls):
            key = spline_obj.shape_key_add(name=name, from_mix=False)
            key.value = 0.0
            key.data[i].radius = 0.0


    @stage_rig_bones
    def rig_spline_object(self):
        for i, info in enumerate(self.all_controls):
            self.rig_spline_hook(i, *info)

        if self.use_radius:
            for i, info in enumerate(self.all_controls):
                self.rig_spline_radius_shapekey(i, *info)

    def rig_spline_hook(self, i, ctrl, subtype, index):
        bone = self.get_bone(ctrl)

        hook = self.spline_obj.modifiers.new(ctrl, 'HOOK')
        hook.object = self.obj
        hook.subtarget = ctrl
        hook.center = bone.head
        hook.vertex_indices_set([i*3, i*3 + 1, i*3 + 2])

        if subtype > 0:
            self.rig_enable_control_driver(hook, 'show_viewport', subtype, index)
            self.rig_enable_control_driver(hook, 'show_render', subtype, index)

            hook = self.spline_obj.modifiers.new(ctrl + "_OFF", 'HOOK')
            hook.object = self.obj
            hook.subtarget = self.tip_controls[subtype]
            hook.center = bone.head
            hook.vertex_indices_set([i*3, i*3 + 1, i*3 + 2])

            self.rig_enable_control_driver(hook, 'show_viewport', subtype, index, disable=True)
            self.rig_enable_control_driver(hook, 'show_render', subtype, index, disable=True)

    def rig_spline_radius_shapekey(self, i, ctrl, subtype, index):
        key = self.spline_obj.data.shape_keys.key_blocks[i + 1]
        switch_prop = self.ENABLE_CONTROL_PROPERTY[subtype]

        assert key.name == ctrl

        scale_expr = 'scale'
        var_map = {
            'scale': driver_var_transform(self.obj, ctrl, type='SCALE_AVG', space='LOCAL')
        }

        if switch_prop:
            base = self.tip_controls[subtype]
            scale_expr += ' if active > %d else base_scale' % (index)
            var_map.update({
                'base_scale': driver_var_transform(self.obj, base, type='SCALE_AVG', space='LOCAL'),
                'active': (self.obj, self.bones.ctrl.master, switch_prop)
            })

        make_driver(
            key, 'value',
            expression='1 - (%s) / %.2f' % (scale_expr, self.max_curve_radius),
            variables=var_map
        )

    ##############################
    # Spline IK Chain

    @stage_generate_bones
    def make_mch_ik_chain(self):
        self.bones.mch.ik = map_list(self.make_mch_ik_bone, self.bones.org)

    def make_mch_ik_bone(self, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '.ik'))

    @stage_parent_bones
    def parent_mch_ik_chain(self):
        self.parent_bone_chain(self.bones.mch.ik, use_connect=True)
        self.set_bone_parent(self.bones.mch.ik[0], self.bones.ctrl.start_twist)

    @stage_rig_bones
    def rig_mch_ik_chain(self):
        for i, args in enumerate(zip(self.bones.mch.ik)):
            self.rig_mch_ik_bone(i, *args)

        self.rig_mch_ik_constraint(self.bones.mch.ik[-1])

    def rig_mch_ik_bone(self, i, mch):
        self.get_bone(mch).rotation_mode = 'XYZ'

        self.make_driver(
            mch, 'rotation_euler', index=1,
            expression = 'var0 / %d' % (len(self.bones.mch.ik)),
            variables=[(self.bones.ctrl.end_twist, '.rotation_euler.y')]
        )

        if self.use_stretch:
            self.make_constraint(
                mch, 'COPY_SCALE', self.bones.mch.end_stretch,
                use_offset=True, space='LOCAL'
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
    # ORG chain

    @stage_parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org[0], self.bones.ctrl.master)

    @stage_rig_bones
    def rig_org_chain(self):
        for args in zip(count(0), self.bones.org, self.bones.mch.ik):
            self.rig_org_bone(*args)

    def rig_org_bone(self, i, org, ik):
        self.make_constraint(org, 'COPY_TRANSFORMS', ik)

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

        params.sik_stretch_control = bpy.props.BoolProperty(
            name="Explicit Squash and Stretch", default=False,
            description="Use the twist controls for squash and stretch, instead of fitting to the curve length"
        )

        params.sik_radius_scaling = bpy.props.BoolProperty(
            name="Radius Scaling", default=True,
            description="Allow scaling the spline control bones to affect the thickness via curve radius"
        )
        params.sik_max_radius = bpy.props.FloatProperty(
            name="Maximum Radius", min=1, default=10,
            description="Maximum supported scale factor for the spline control bones"
        )


    @classmethod
    def parameters_ui(self, layout, params):
        """ Create the ui for the rig parameters. """

        layout.label(icon='INFO', text='A straight line rest shape works best.')

        layout.prop(params, 'sik_start_controls')
        layout.prop(params, 'sik_mid_controls')
        layout.prop(params, 'sik_end_controls')

        layout.prop(params, 'sik_stretch_control')

        layout.prop(params, 'sik_radius_scaling')

        col = layout.column()
        col.active = params.sik_radius_scaling
        col.prop(params, 'sik_max_radius')


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
