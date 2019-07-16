#====================== BEGIN GPL LICENSE BLOCK ======================
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#======================= END GPL LICENSE BLOCK ========================

# <pep8 compliant>

import bpy

from rigify.utils.animation import add_generic_snap_fk_to_ik
from rigify.utils.rig import connected_children_names
from rigify.utils.bones import BoneDict, put_bone, compute_chain_x_axis, align_bone_x_axis, align_bone_orientation
from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.layers import ControlLayersOption
from rigify.utils.misc import pairwise_nozip, padnone, map_list
from rigify.utils.switch_parent import SwitchParentBuilder

from rigify.base_rig import stage, BaseRig

from rigify.utils.widgets_basic import create_circle_widget, create_sphere_widget, create_line_widget, create_limb_widget
from rigify.rigs.widgets import create_gear_widget, create_ikarrow_widget

from math import pi
from itertools import count, repeat, chain
from mathutils import Vector
from collections import namedtuple


SegmentEntry = namedtuple('SegmentEntry', ['org', 'org_idx', 'seg_idx', 'pos'])


class BaseLimbRig(BaseRig):
    """Common base for limb rigs."""

    segmented_orgs = 2 # Number of org bones to segment

    def find_org_bones(self, bone):
        return BoneDict(
            main=[bone.name] + connected_children_names(self.obj, bone.name),
        )

    def initialize(self):
        orgs = self.bones.org.main

        if len(orgs) < self.segmented_orgs + 1:
            self.raise_error("Input to rig type must be a chain of at least 3 bones.")

        self.segments = self.params.segments
        self.bbone_segments = self.params.bbones

        self.segment_table = [
            SegmentEntry(org, i, j, self.get_segment_pos(org, j))
            for i, org in enumerate(orgs[:self.segmented_orgs])
            for j in range(self.segments)
        ]
        self.segment_table_end = [
            SegmentEntry(org, i + self.segmented_orgs, None, self.get_bone(org).head)
            for i, org in enumerate(orgs[self.segmented_orgs:])
        ]

        self.segment_table_full = self.segment_table + self.segment_table_end
        self.segment_table_tweak = self.segment_table + self.segment_table_end[0:1]

    def generate_bones(self):
        bones = map_list(self.get_bone, self.bones.org.main[0:2])

        self.elbow_vector = self.compute_elbow_vector(bones)
        self.pole_angle = self.compute_pole_angle(bones, self.elbow_vector)
        self.rig_parent_bone = self.get_bone_parent(self.bones.org.main[0])


    ####################################################
    # Utilities

    def compute_elbow_vector(self, bones):
        lo_vector = bones[1].vector
        tot_vector = bones[1].tail - bones[0].head
        return (lo_vector.project(tot_vector) - lo_vector).normalized() * tot_vector.length

    def compute_pole_angle(self, bones, elbow_vector):
        rot_axis = self.params.rotation_axis

        if rot_axis == 'z':
            return 0

        if rot_axis == 'x' or rot_axis == 'automatic':
            z_vector = bones[0].z_axis + bones[1].z_axis
            alfa = elbow_vector.angle(z_vector)
        elif rot_axis == 'z':
            x_vector = bones[0].x_axis + bones[1].x_axis
            alfa = elbow_vector.angle(x_vector)
        else:
            self.raise_error('Unexpected axis value: {}', rot_axis)

        if alfa > pi/2:
            return -pi/2
        else:
            return pi/2

    def get_segment_pos(self, org, seg):
        bone = self.get_bone(org)
        return bone.head + bone.vector * (seg / self.segments)

    @staticmethod
    def vector_without_z(vector):
        return Vector((vector[0], vector[1], 0))

    ####################################################
    # BONES
    #
    # org:
    #   main[]:
    #     Main ORG bone chain
    # ctrl:
    #   master:
    #     Main property control.
    #   fk[]:
    #     FK control chain.
    #   tweak[]:
    #     Tweak control chain.
    #   ik_base, ik_pole, ik
    #     IK controls
    #   ik_vispole
    #     IK pole visualization.
    # mch:
    #   master:
    #     Parent of the master control.
    #   follow:
    #     FK follow behavior.
    #   fk[]:
    #     FK chain parents (or None)
    #   ik_stretch
    #     IK stretch switch implementation.
    #   ik_target
    #     Corrected target position.
    #   ik_end
    #     End of the IK chain: [ik_base, ik_end]
    # deform[]:
    #   DEF bones
    #
    ####################################################

    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control_bone(self):
        org = self.bones.org.main[0]
        self.bones.mch.master = name = self.copy_bone(org, make_derived_name(org, 'mch', '_parent_socket'), scale=1/12)
        self.get_bone(name).roll = 0
        self.bones.ctrl.master = name = self.copy_bone(org, make_derived_name(org, 'ctrl', '_parent'), scale=1/4)
        self.get_bone(name).roll = 0
        self.prop_bone = self.bones.ctrl.master

    @stage.parent_bones
    def parent_master_control_bone(self):
        self.set_bone_parent(self.bones.ctrl.master, self.bones.mch.master)
        self.set_bone_parent(self.bones.mch.master, self.bones.mch.follow)

    @stage.configure_bones
    def configure_master_control_bone(self):
        bone = self.get_bone(self.bones.ctrl.master)
        bone.lock_location = (True, True, True)
        bone.lock_rotation = (True, True, True)
        bone.lock_scale = (True, True, True)
        bone.lock_rotation_w = True

    @stage.rig_bones
    def rig_master_control_bone(self):
        self.make_constraint(self.bones.mch.master, 'COPY_ROTATION', self.bones.org.main[0])

    @stage.generate_widgets
    def make_master_control_widget(self):
        create_gear_widget(self.obj, self.bones.ctrl.master, size=10)


    ####################################################
    # FK follow MCH

    @stage.generate_bones
    def make_mch_follow_bone(self):
        org = self.bones.org.main[0]
        self.bones.mch.follow = self.copy_bone(org, make_derived_name(org, 'mch', '_parent'), scale=1/4)

    @stage.parent_bones
    def parent_mch_follow_bone(self):
        mch = self.bones.mch.follow
        align_bone_orientation(self.obj, mch, 'root')
        self.set_bone_parent(mch, self.rig_parent_bone)

    @stage.configure_bones
    def configure_mch_follow_bone(self):
        ctrl = self.bones.ctrl
        panel = self.script.panel_with_selected_check(self, [ctrl.master, *ctrl.fk])

        self.make_property(self.prop_bone, 'FK_limb_follow', default=0.0)
        panel.custom_prop(self.prop_bone, 'FK_limb_follow', text='FK Limb Follow', slider=True)

    @stage.rig_bones
    def rig_mch_follow_bone(self):
        mch = self.bones.mch.follow

        con = self.make_constraint(mch, 'COPY_ROTATION', 'root')
        self.make_constraint(mch, 'COPY_SCALE', 'root')

        self.make_driver(con, 'influence', variables=[(self.prop_bone, 'FK_limb_follow')])


    ####################################################
    # FK control chain

    @stage.generate_bones
    def make_fk_control_chain(self):
        self.bones.ctrl.fk = map_list(self.make_fk_control_bone, count(0), self.bones.org.main)

    def get_fk_name(self, i, org, kind):
        return make_derived_name(org, kind, '_fk' if i <= 2 else '')

    def make_fk_control_bone(self, i, org):
        return self.copy_bone(org, self.get_fk_name(i, org, 'ctrl'))

    @stage.parent_bones
    def parent_fk_control_chain(self):
        fk = self.bones.ctrl.fk
        for args in zip(count(0), fk, [self.bones.mch.follow]+fk, self.bones.org.main, self.bones.mch.fk):
            self.parent_fk_control_bone(*args)

    def parent_fk_control_bone(self, i, ctrl, prev, org, parent_mch):
        if parent_mch:
            self.set_bone_parent(ctrl, parent_mch)
        else:
            self.set_bone_parent(ctrl, prev, use_connect=(i > 0))

    @stage.configure_bones
    def configure_fk_control_chain(self):
        for args in zip(count(0), self.bones.ctrl.fk, self.bones.org.main):
            self.configure_fk_control_bone(*args)

        ControlLayersOption.FK.assign(self.params, self.obj, self.bones.ctrl.fk)

    def configure_fk_control_bone(self, i, ctrl, org):
        self.copy_bone_properties(org, ctrl)

        if 2 <= i <= self.segmented_orgs:
            self.get_bone(ctrl).lock_location = True, True, True

    @stage.generate_widgets
    def make_fk_control_widgets(self):
        for args in zip(count(0), self.bones.ctrl.fk):
            self.make_fk_control_widget(*args)

    def make_fk_control_widget(self, i, ctrl):
        if i < 2:
            create_limb_widget(self.obj, ctrl)
        else:
            create_circle_widget(self.obj, ctrl, radius=0.4, head_tail=0.0)


    ####################################################
    # FK parents MCH chain

    @stage.generate_bones
    def make_fk_parent_chain(self):
        self.bones.mch.fk = map_list(self.make_fk_parent_bone, count(0), self.bones.org.main)

    def make_fk_parent_bone(self, i, org):
        if 2 <= i <= 3:
            return self.copy_bone(org, self.get_fk_name(i, org, 'mch'), parent=True, scale=1/4)

    @stage.parent_bones
    def parent_fk_parent_chain(self):
        mch = self.bones.mch
        for args in zip(count(0), mch.fk, [mch.follow]+self.bones.ctrl.fk, self.bones.org.main):
            self.parent_fk_parent_bone(*args)

    def parent_fk_parent_bone(self, i, parent_mch, prev_ctrl, org):
        if i == 2:
            self.set_bone_parent(parent_mch, prev_ctrl, use_connect=True)

    @stage.rig_bones
    def rig_fk_parent_chain(self):
        for args in zip(count(0), self.bones.mch.fk, self.bones.org.main):
            self.rig_fk_parent_bone(*args)

    def rig_fk_parent_bone(self, i, parent_mch, org):
        if i == 2:
            self.make_constraint(parent_mch, 'COPY_SCALE', 'root')


    ####################################################
    # IK controls

    @stage.generate_bones
    def make_ik_controls(self):
        orgs = self.bones.org.main

        self.bones.ctrl.ik_base = self.copy_bone(orgs[0], make_derived_name(orgs[0], 'ctrl', '_ik'))
        self.bones.ctrl.ik_pole = self.make_ik_pole_bone(orgs)
        self.bones.ctrl.ik = self.make_ik_control_bone(orgs)

        self.build_ik_parent_switch(SwitchParentBuilder(self.generator))

    def make_ik_pole_bone(self, orgs):
        name = self.copy_bone(orgs[0], make_derived_name(orgs[0], 'ctrl', '_ik_target'))

        pole = self.get_bone(name)
        pole.head = self.get_bone(orgs[0]).tail + self.elbow_vector
        pole.tail = pole.head - self.elbow_vector/8
        pole.roll = 0

        return name

    def make_ik_control_bone(self, orgs):
        return self.copy_bone(orgs[2], make_derived_name(orgs[2], 'ctrl', '_ik'))

    def build_ik_parent_switch(self, pbuilder):
        ctrl = self.bones.ctrl

        prop_bone = lambda: ctrl.master
        pcontrols = lambda: [ ctrl.master, ctrl.ik_base, ctrl.ik_pole, ctrl.ik ]

        if self.rig_parent_bone:
            pbuilder.register_parent(self, self.rig_parent_bone)

        pbuilder.build_child(
            self, ctrl.ik,
            prop_bone=prop_bone, prop_id='IK_parent', prop_name='IK Parent',
            controls=pcontrols,
        )

        pbuilder.build_child(
            self, ctrl.ik_pole, extra_parents=[ctrl.ik],
            prop_bone=prop_bone, prop_id='pole_parent', prop_name='Pole Parent',
            controls=pcontrols, no_fix_rotation=True, no_fix_scale=True,
        )

    @stage.configure_bones
    def configure_ik_controls(self):
        base = self.get_bone(self.bones.ctrl.ik_base)
        base.rotation_mode = 'ZXY'
        base.lock_rotation = True, False, True
        base.ik_stretch = 0.1

    @stage.rig_bones
    def rig_ik_controls(self):
        self.rig_hide_pole_control(self.bones.ctrl.ik_pole)

    @stage.generate_widgets
    def make_ik_control_widgets(self):
        self.make_ik_base_widget(self.bones.ctrl.ik_base)
        self.make_ik_pole_widget(self.bones.ctrl.ik_pole)
        self.make_ik_ctrl_widget(self.bones.ctrl.ik)

    def make_ik_base_widget(self, ctrl):
        if self.params.rotation_axis in {'x', 'automatic'}:
            roll = 0
        else:
            roll = pi/2

        create_ikarrow_widget(self.obj, ctrl, bone_transform_name=None, roll=roll)

    def make_ik_pole_widget(self, ctrl):
        create_sphere_widget(self.obj, ctrl, bone_transform_name=None)

    def make_ik_ctrl_widget(self, ctrl):
        self.raise_error('NOT IMPLEMENTED')


    ####################################################
    # IK pole visualization

    @stage.generate_bones
    def make_ik_vispole_bone(self):
        orgs = self.bones.org.main
        name = self.copy_bone(orgs[1], 'VIS_'+make_derived_name(orgs[0], 'ctrl', '_ik_pole'))
        self.bones.ctrl.ik_vispole = name

        bone = self.get_bone(name)
        bone.tail = bone.head + Vector((0, 0, bone.length / 10))
        bone.hide_select = True

    @stage.rig_bones
    def rig_ik_vispole_bone(self):
        name = self.bones.ctrl.ik_vispole

        self.make_constraint(name, 'COPY_LOCATION', self.bones.org.main[1])
        self.make_constraint(
            name, 'STRETCH_TO', self.bones.ctrl.ik_pole,
            volume='NO_VOLUME', rest_length=self.get_bone(name).length
        )

        self.rig_hide_pole_control(name)

    @stage.generate_widgets
    def make_ik_vispole_widget(self):
        create_line_widget(self.obj, self.bones.ctrl.ik_vispole)


    ####################################################
    # IK system MCH

    def get_ik_input_bone(self):
        return self.bones.ctrl.ik

    def get_ik_output_chain(self):
        return [self.bones.ctrl.ik_base, self.bones.mch.ik_end, self.bones.mch.ik_target]

    def get_all_ik_controls(self):
        ctrl = self.bones.ctrl
        return [ctrl.ik_base, ctrl.ik_pole, ctrl.ik]

    @stage.generate_bones
    def make_ik_mch_chain(self):
        orgs = self.bones.org.main
        self.bones.mch.ik_stretch = self.make_ik_mch_stretch_bone(orgs)
        self.bones.mch.ik_target = self.make_ik_mch_target_bone(orgs)
        self.bones.mch.ik_end = self.copy_bone(orgs[1], make_derived_name(orgs[1], 'mch', '_ik'))

    def make_ik_mch_stretch_bone(self, orgs):
        name = self.copy_bone(orgs[0], make_derived_name(orgs[0], 'mch', '_ik_stretch'))
        self.get_bone(name).tail = self.get_bone(orgs[2]).head
        return name

    def make_ik_mch_target_bone(self, orgs):
        return self.copy_bone(orgs[2], make_derived_name(orgs[0], 'mch', '_ik_target'))

    @stage.parent_bones
    def parent_ik_mch_chain(self):
        self.set_bone_parent(self.bones.mch.ik_stretch, self.bones.mch.follow)
        self.set_bone_parent(self.bones.mch.ik_target, self.get_ik_input_bone())
        self.set_bone_parent(self.bones.mch.ik_end, self.bones.ctrl.ik_base)

    @stage.configure_bones
    def configure_ik_mch_chain(self):
        ctrl = self.bones.ctrl
        panel = self.script.panel_with_selected_check(self, ctrl.flatten())

        name = strip_org(self.bones.org.main[2])

        self.make_property(self.prop_bone, 'IK_FK', default=0.0, description='IK/FK Switch')
        panel.custom_prop(self.prop_bone, 'IK_FK', text='IK-FK ({})'.format(name), slider=True)

        self.add_ik_snap_buttons(panel, name)

        panel = self.script.panel_with_selected_check(self, [ctrl.master, *self.get_all_ik_controls()])

        self.make_property(self.prop_bone, 'IK_Stretch', default=1.0, description='IK Stretch')
        panel.custom_prop(self.prop_bone, 'IK_Stretch', text='IK Stretch', slider=True)

        self.make_property(self.prop_bone, 'pole_vector', default=False, description='Pole Vector')
        panel.custom_prop(self.prop_bone, 'pole_vector', text='Pole Vector')

    def add_ik_snap_buttons(self, panel, rig_name):
        ik_chain = self.get_ik_output_chain()
        add_generic_snap_fk_to_ik(
            panel,
            fk_bones=self.bones.ctrl.fk[0:len(ik_chain)],
            ik_bones=ik_chain,
            ik_ctrl_bones=self.get_all_ik_controls(),
            rig_name=rig_name
        )

    @stage.rig_bones
    def rig_ik_mch_chain(self):
        mch = self.bones.mch
        input_bone = self.get_ik_input_bone()

        self.rig_ik_mch_stretch_bone(mch.ik_stretch, input_bone)
        self.rig_ik_mch_target_bone(mch.ik_target, mch.ik_stretch, input_bone)
        self.rig_ik_mch_end_bone(mch.ik_end, mch.ik_target, self.bones.ctrl.ik_pole)

    def rig_ik_mch_stretch_bone(self, mch_stretch, input_bone):
        self.make_constraint(mch_stretch, 'DAMPED_TRACK', input_bone)
        self.make_constraint(mch_stretch, 'STRETCH_TO', input_bone)

        con = self.make_constraint(mch_stretch, 'LIMIT_SCALE', min_y=0.0, max_y=1.05, owner_space='LOCAL')

        self.make_driver(con, "influence", variables=[(self.prop_bone, 'IK_Stretch')], polynomial=[1.0, -1.0])

    def rig_ik_mch_target_bone(self, mch_target, mch_stretch, input_bone):
        self.make_constraint(mch_target, 'COPY_LOCATION', mch_stretch, head_tail=1.0)

    def rig_ik_mch_end_bone(self, mch_ik, mch_target, ctrl_pole):
        bone = self.get_bone(mch_ik)
        bone.ik_stretch = 0.1

        if self.params.rotation_axis == 'z':
            bone.lock_ik_x = True
            bone.lock_ik_y = True
        else:
            bone.lock_ik_y = True
            bone.lock_ik_z = True

        con = self.make_constraint(
            mch_ik, 'IK', mch_target, chain_count=2,
        )

        self.make_driver(con, "mute", variables=[(self.prop_bone, 'pole_vector')], polynomial=[0.0, 1.0])

        con_pole = self.make_constraint(
            mch_ik, 'IK', mch_target, chain_count=2,
            pole_target=self.obj, pole_subtarget=ctrl_pole, pole_angle=self.pole_angle,
        )

        self.make_driver(con_pole, "mute", variables=[(self.prop_bone, 'pole_vector')], polynomial=[1.0, -1.0])

    def rig_hide_pole_control(self, name):
        self.make_driver(
            self.get_bone(name).bone, "hide",
            variables=[(self.prop_bone, 'pole_vector')], polynomial=[1.0, -1.0],
        )


    ####################################################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        pass

    @stage.rig_bones
    def rig_org_chain(self):
        ik = self.get_ik_output_chain()
        for args in zip(count(0), self.bones.org.main, self.bones.ctrl.fk, padnone(ik)):
            self.rig_org_bone(*args)

    def rig_org_bone(self, i, org, fk, ik):
        self.make_constraint(org, 'COPY_TRANSFORMS', fk)

        if ik:
            con = self.make_constraint(org, 'COPY_TRANSFORMS', ik)

            self.make_driver(con, 'influence', variables=[(self.prop_bone, 'IK_FK')], polynomial=[1.0, -1.0])


    ####################################################
    # Tweak control chain

    @stage.generate_bones
    def make_tweak_chain(self):
        self.bones.ctrl.tweak = map_list(self.make_tweak_bone, count(0), self.segment_table_tweak)

    def make_tweak_bone(self, i, entry):
        name = make_derived_name(entry.org, 'ctrl', '_tweak')
        name = self.copy_bone(entry.org, name, scale=1/(2 * self.segments))
        put_bone(self.obj, name, entry.pos)
        return name

    @stage.parent_bones
    def parent_tweak_chain(self):
        for ctrl, mch in zip(self.bones.ctrl.tweak, self.bones.mch.tweak):
            self.set_bone_parent(ctrl, mch)

    @stage.configure_bones
    def configure_tweak_chain(self):
        for args in zip(count(0), self.bones.ctrl.tweak, self.segment_table_tweak):
            self.configure_tweak_bone(*args)

        ControlLayersOption.TWEAK.assign(self.params, self.obj, self.bones.ctrl.tweak)

    def configure_tweak_bone(self, i, tweak, entry):
        tweak_pb = self.get_bone(tweak)
        tweak_pb.lock_rotation = (True, False, True)
        tweak_pb.lock_scale = (False, True, False)
        tweak_pb.rotation_mode = 'ZXY'

        if i > 0 and entry.seg_idx is not None:
            self.make_rubber_tweak_property(i, tweak, entry)

    def make_rubber_tweak_property(self, i, tweak, entry):
        defval = 1.0 if entry.seg_idx else 0.0
        text = 'Rubber Tweak ({})'.format(strip_org(entry.org))

        self.make_property(tweak, 'rubber_tweak', defval, max=2.0, soft_max=1.0)

        panel = self.script.panel_with_selected_check(self, [tweak])
        panel.custom_prop(tweak, 'rubber_tweak', text=text, slider=True)

    @stage.generate_widgets
    def make_tweak_widgets(self):
        for tweak in self.bones.ctrl.tweak:
            self.make_tweak_widget(tweak)

    def make_tweak_widget(self, tweak):
        create_sphere_widget(self.obj, tweak)


    ####################################################
    # Tweak MCH chain

    @stage.generate_bones
    def make_tweak_mch_chain(self):
        self.bones.mch.tweak = map_list(self.make_tweak_mch_bone, count(0), self.segment_table_tweak)

    def make_tweak_mch_bone(self, i, entry):
        name = make_derived_name(entry.org, 'mch', '_tweak')
        name = self.copy_bone(entry.org, name, scale=1/(4 * self.segments))
        put_bone(self.obj, name, entry.pos)
        return name

    @stage.parent_bones
    def parent_tweak_mch_chain(self):
        for mch, entry in zip(self.bones.mch.tweak, self.segment_table_tweak):
            self.set_bone_parent(mch, entry.org)

    @stage.rig_bones
    def rig_tweak_mch_chain(self):
        for args in zip(count(0), self.bones.mch.tweak, self.segment_table_tweak):
            self.rig_tweak_mch_bone(*args)

    def rig_tweak_mch_bone(self, i, tweak, entry):
        if entry.seg_idx:
            tweaks = self.bones.ctrl.tweak
            prev_tweak = tweaks[i - entry.seg_idx]
            next_tweak = tweaks[i + self.segments - entry.seg_idx]

            self.make_constraint(tweak, 'COPY_TRANSFORMS', prev_tweak)
            self.make_constraint(
                tweak, 'COPY_TRANSFORMS', next_tweak,
                influence = entry.seg_idx / self.segments
            )
            self.make_constraint(tweak, 'DAMPED_TRACK', next_tweak)

        elif entry.seg_idx is not None:
            self.make_constraint(tweak, 'COPY_SCALE', 'root')


    ####################################################
    # Deform chain

    @stage.generate_bones
    def make_deform_chain(self):
        self.bones.deform = map_list(self.make_deform_bone, count(0), self.segment_table_full)

    def make_deform_bone(self, i, entry):
        name = make_derived_name(entry.org, 'def')

        if entry.seg_idx is None:
            name = self.copy_bone(entry.org, name)
        else:
            name = self.copy_bone(entry.org, name, bbone=True, scale=1/self.segments)
            put_bone(self.obj, name, entry.pos)
            self.get_bone(name).bbone_segments = self.bbone_segments

        return name

    @stage.parent_bones
    def parent_deform_chain(self):
        self.set_bone_parent(self.bones.deform[0], self.rig_parent_bone)
        self.parent_bone_chain(self.bones.deform, use_connect=True)

    @stage.rig_bones
    def rig_deform_chain(self):
        tweaks = pairwise_nozip(padnone(self.bones.ctrl.tweak))
        entries = pairwise_nozip(padnone(self.segment_table_full))

        for args in zip(count(0), self.bones.deform, *entries, *tweaks):
            self.rig_deform_bone(*args)

    def rig_deform_bone(self, i, deform, entry, next_entry, tweak, next_tweak):
        if tweak:
            self.make_constraint(deform, 'COPY_TRANSFORMS', tweak)

            if next_tweak:
                self.make_constraint(deform, 'DAMPED_TRACK', next_tweak)
                self.make_constraint(deform, 'STRETCH_TO', next_tweak)

                self.rig_deform_easing(i, deform, tweak, next_tweak)

            elif next_entry:
                self.make_constraint(deform, 'DAMPED_TRACK', next_entry.org)
                self.make_constraint(deform, 'STRETCH_TO', next_entry.org)
        else:
            self.make_constraint(deform, 'COPY_TRANSFORMS', entry.org)

    def rig_deform_easing(self, i, deform, tweak, next_tweak):
        pbone = self.get_bone(deform)

        if 'rubber_tweak' in self.get_bone(tweak):
            self.make_driver(pbone.bone, 'bbone_easein', variables=[(tweak, 'rubber_tweak')])
        else:
            pbone.bone.bbone_easein = 0.0

        if 'rubber_tweak' in self.get_bone(next_tweak):
            self.make_driver(pbone.bone, 'bbone_easeout', variables=[(next_tweak, 'rubber_tweak')])
        else:
            pbone.bone.bbone_easeout = 0.0


    ####################################################
    # Settings

    @classmethod
    def add_parameters(self, params):
        """ Add the parameters of this rig type to the
            RigifyParameters PropertyGroup
        """

        items = [
            ('x', 'X manual', ''),
            ('z', 'Z manual', ''),
            ('automatic', 'Automatic', '')
        ]

        params.rotation_axis = bpy.props.EnumProperty(
            items   = items,
            name    = "Rotation Axis",
            default = 'automatic'
        )

        params.auto_align_extremity = bpy.BoolProperty(
            name='auto_align_extremity',
            default=False,
            description="Auto Align Extremity Bone"
        )

        params.segments = bpy.props.IntProperty(
            name        = 'limb segments',
            default     = 2,
            min         = 1,
            description = 'Number of segments'
        )

        params.bbones = bpy.props.IntProperty(
            name        = 'bbone segments',
            default     = 10,
            min         = 1,
            description = 'Number of segments'
        )

        # Setting up extra layers for the FK and tweak
        ControlLayersOption.FK.add_parameters(params)
        ControlLayersOption.TWEAK.add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params, end='End'):
        """ Create the ui for the rig parameters."""

        r = layout.row()
        r.prop(params, "rotation_axis")

        if 'auto' not in params.rotation_axis.lower():
            r = layout.row()
            r.prop(params, "auto_align_extremity", text="Auto Align "+end)

        r = layout.row()
        r.prop(params, "segments")

        r = layout.row()
        r.prop(params, "bbones")

        ControlLayersOption.FK.parameters_ui(layout, params)
        ControlLayersOption.TWEAK.parameters_ui(layout, params)
