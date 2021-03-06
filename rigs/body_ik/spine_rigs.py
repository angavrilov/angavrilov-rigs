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
import json

from math import pi
from itertools import count, repeat, chain
from mathutils import Vector

from rigify.utils.animation import SCRIPT_UTILITIES_BAKE
from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.misc import map_list
from rigify.utils.bones import put_bone, set_bone_widget_transform

from rigify.base_rig import stage

from rigify.rigs.spines import spine_rigs


class BaseBodyIkSpineRig(spine_rigs.BaseSpineRig):
    def initialize(self):
        from . import limb_rigs

        super().initialize()

        orgs = self.bones.org

        legs = [
            child for child in self.rigify_children
            if (isinstance(child, limb_rigs.BaseBodyIkLegRig) and
                self.get_bone_parent(child.bones.org.main[0]) == orgs[0])
        ]

        if len(legs) != 2:
            self.raise_error('IK spine requires two legs attached to the hip bone.')

        self.leg_rigs = legs

    ####################################################
    # BONES
    #
    # mch:
    #   hip_output:
    #     Output of hip IK correction.
    #   hip_offset:
    #     Global position represents the offset applied by Hip IK.
    #   last_tweak_offset:
    #     Offset bone for the last tweak. With the ORG bones forms
    #     the corrected tweak chain.
    #   leg_offset[]:
    #     Offset bones between leg base and hip base.
    #   hip_ik[], hip_ik_tgt:
    #     Chain for solving the two point limit distance problem.
    #
    ####################################################

    ####################################################
    # Master control bone

    @stage.configure_bones
    def configure_master_control(self):
        super().configure_master_control()

        leg_controls = list(chain.from_iterable(leg.get_snap_body_ik_controls() for leg in self.leg_rigs))

        panel = self.script.panel_with_selected_check(self, self.bones.ctrl.flatten() + leg_controls)

        self.generate_body_ik_panel(panel)

    def generate_body_ik_panel(self, panel):
        add_spine_ik_snap(
            panel,
            master=self.bones.ctrl.master,
            result=self.get_result_bone(),
            final=self.bones.org[0]
        )

    def get_result_bone(self):
        return self.bones.ctrl.tweak[0]

    ####################################################
    # Leg offset MCH bones

    @stage.generate_bones
    def make_leg_offset_mch_bones(self):
        self.bones.mch.leg_offset = map_list(self.make_leg_offset_mch_bone, count(0), self.leg_rigs)

    def make_leg_offset_mch_bone(self, i, leg_rig):
        org = leg_rig.bones.org.main[0]
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik'))
        self.get_bone(name).tail = self.get_bone(self.bones.org[0]).head
        return name

    @stage.parent_bones
    def parent_leg_offset_mch_bones(self):
        for mch in self.bones.mch.leg_offset:
            self.set_bone_parent(mch, self.get_result_bone())

    @stage.configure_bones
    def configure_leg_offset_mch_bones(self):
        for args in zip(count(0), self.leg_rigs, self.bones.mch.leg_offset):
            self.configure_leg_offset_mch_bone(*args)

    def configure_leg_offset_mch_bone(self, i, leg_rig, mch):
        bone = self.get_bone(mch)
        bone['length'] = 1.0
        bone['influence'] = 0.0

    @stage.rig_bones
    def rig_leg_offset_mch_bones(self):
        for rig, mch in zip(self.leg_rigs, self.bones.mch.leg_offset):
            rig.rig_body_ik_target_bone(mch)

    ####################################################
    # Hip IK MCH chain

    @stage.generate_bones
    def make_hip_ik_mch_chain(self):
        org = self.bones.org[0]
        self.bones.mch.hip_ik_tgt = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik_tgt'), scale=1/5)
        mch1 = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik'))
        mch2 = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik_end'))
        bone1 = self.get_bone(mch1)
        bone2 = self.get_bone(mch2)
        bone1.tail = bone2.head = bone1.head + Vector((0, 0, 1))
        bone2.tail = bone1.head + Vector((1, 0, 1))
        bone1.roll = bone2.roll = 0
        bone2.use_inherit_scale = False
        self.bones.mch.hip_ik = [ mch1, mch2 ]

    @stage.parent_bones
    def parent_hip_ik_mch_chain(self):
        mch = self.bones.mch
        self.generator.disable_auto_parent(mch.hip_ik[0])
        self.parent_bone_chain(mch.hip_ik, use_connect=True)
        self.set_bone_parent(mch.hip_ik_tgt, mch.leg_offset[1], use_connect=True)

    @stage.rig_bones
    def rig_hip_ik_mch_chain(self):
        mch = self.bones.mch

        for args in zip(count(0), mch.hip_ik, mch.leg_offset):
            self.rig_hip_ik_mch_bone(*args)

        self.make_constraint(mch.hip_ik[0], 'COPY_LOCATION', mch.leg_offset[0], head_tail=1)
        self.rig_hip_ik_system(mch.hip_ik[0], mch.hip_ik[1], mch.hip_ik_tgt, self.get_result_bone())

    def rig_hip_ik_mch_bone(self, i, mch_ik, mch_in):
        self.make_driver(mch_ik, 'scale', index=1, variables=[(mch_in, 'length')])

    def rig_hip_ik_system(self, mch_base, mch_ik, mch_tgt, pole):
        '''
        self.make_constraint(mch_base, 'DAMPED_TRACK', pole)
        self.make_constraint(
            mch_base, 'LOCKED_TRACK', mch_tgt,
            lock_axis='LOCK_Y', track_axis='TRACK_X',
        )
        '''

        bone_ik = self.get_bone(mch_ik)
        bone_ik.lock_ik_y = bone_ik.lock_ik_z = True
        bone_ik.use_ik_limit_x = True
        bone_ik.ik_min_x = -pi/2
        bone_ik.ik_max_x = pi/2

        self.make_constraint(
            mch_ik, 'IK', mch_tgt,
            pole_target=self.obj, pole_subtarget=pole, pole_angle=pi,
            chain_count=2, use_stretch=False,
        )

    def rig_hip_ik_output(self, out, lim_both, lim_in1, lim_in2):
        inf_vars = {'inf1':(lim_in1, 'influence'), 'inf2':(lim_in2, 'influence')}

        con = self.make_constraint(out, 'COPY_LOCATION', lim_both)
        self.make_driver(con, 'influence', variables=inf_vars, expression='min(inf1,inf2)')

        con = self.make_constraint(
            out, 'LIMIT_DISTANCE', lim_in1, head_tail=1, space='POSE',
            limit_mode='LIMITDIST_ONSURFACE', distance=1,
        )
        self.make_driver(con, 'distance', variables=[(lim_in1, 'length')])
        self.make_driver(
            con, 'influence', variables=inf_vars,
            expression='(inf1-inf2)/(1-inf2) if inf1 > inf2 else 0'
        )

        con = self.make_constraint(
            out, 'LIMIT_DISTANCE', lim_in2, head_tail=1, space='POSE',
            limit_mode='LIMITDIST_ONSURFACE', distance=1,
        )
        self.make_driver(con, 'distance', variables=[(lim_in2, 'length')])
        self.make_driver(
            con, 'influence', variables=inf_vars,
            expression='(inf2-inf1)/(1-inf1) if inf2 > inf1 else 0'
        )

    ####################################################
    # Hip offset bones

    @stage.generate_bones
    def make_hip_offset_bones(self):
        org = self.bones.org
        mch = self.bones.mch
        mch.hip_output = self.copy_bone(org[0], make_derived_name(org[0], 'mch', '.hip_output'), scale=0.25)
        mch.hip_offset = self.copy_bone(org[0], make_derived_name(org[0], 'mch', '.hip_offset'), scale=0.20)

    @stage.parent_bones
    def parent_hip_offset_bones(self):
        mch = self.bones.mch
        self.set_bone_parent(mch.hip_output, self.get_result_bone())
        self.set_bone_parent(mch.hip_offset, mch.hip_output)

    @stage.rig_bones
    def rig_hip_offset_bones(self):
        mch = self.bones.mch

        self.rig_hip_ik_output(mch.hip_output, mch.hip_ik[1], mch.leg_offset[0], mch.leg_offset[1])
        self.make_constraint(mch.hip_offset, 'COPY_LOCATION', self.get_hip_offset_base_bone(), invert_xyz=(True,True,True), use_offset=True, space='POSE')

    def get_hip_offset_base_bone(self):
        return self.get_result_bone()

    ####################################################
    # Apply offsets

    @stage.generate_bones
    def make_last_tweak_offset_bone(self):
        org = self.bones.org
        mch = self.bones.mch
        mch.last_tweak_offset = self.copy_bone(org[-1], make_derived_name(org[-1], 'mch', '.end_offset'), scale=0.5)
        put_bone(self.obj, mch.last_tweak_offset, self.get_bone(org[-1]).tail)

    @stage.parent_bones
    def parent_last_tweak_offset_bone(self):
        self.set_bone_parent(self.bones.mch.last_tweak_offset, self.bones.ctrl.tweak[-1])

    @stage.rig_bones
    def rig_last_tweak_offset_bone(self):
        mch = self.bones.mch
        self.make_constraint(mch.last_tweak_offset, 'COPY_LOCATION', mch.hip_offset, use_offset=True, space='POSE')

    def rig_org_bone(self, i, org, tweak, next_tweak):
        self.make_constraint(org, 'COPY_LOCATION', self.bones.mch.hip_offset, use_offset=True, space='POSE')

    @stage.rig_bones
    def rig_deform_chain(self):
        inputs = [*self.bones.org, self.bones.mch.last_tweak_offset]
        for args in zip(count(0), self.bones.deform, inputs, inputs[1:]):
            self.rig_deform_bone(*args)

    '''
    @stage.generate_widgets
    def make_tweak_widgets(self):
        tweaks = self.bones.ctrl.tweak

        super().make_tweak_widgets()

        for tweak, org in zip(tweaks, self.bones.org):
            set_bone_widget_transform(self.obj, tweak, org)

        set_bone_widget_transform(self.obj, tweaks[-1], self.bones.mch.last_tweak_offset)
    '''


##########################
# Snap master to hip IK ##
##########################

SCRIPT_REGISTER_OP_SNAP = ['POSE_OT_rigify_spine_ik_snap', 'POSE_OT_rigify_spine_ik_snap_bake']

SCRIPT_UTILITIES_OP_SNAP = ['''
##########################
## Spine Snap To Hip IK ##
##########################

class RigifySpineIkSnapBase:
    master_bone:  StringProperty(name="Master Control")
    result_bone:  StringProperty(name="Result Control")
    final_bone:   StringProperty(name="Final Control")

    keyflags = None

    def save_frame_state(self, context, obj):
        bones = obj.pose.bones
        delta = bones[self.final_bone].head - bones[self.result_bone].head
        return bones[self.master_bone].head + delta

    def apply_frame_state(self, context, obj, pos):
        matrix = Matrix(obj.pose.bones[self.master_bone].matrix)
        matrix.translation = pos

        set_transform_from_matrix(obj, self.master_bone, matrix, no_rot=True, no_scale=True, keyflags=self.keyflags)

class POSE_OT_rigify_spine_ik_snap(RigifySpineIkSnapBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_spine_ik_snap_" + rig_id
    bl_label = "Snap To Hip IK"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the spine control to corrected Hip IK result"

class POSE_OT_rigify_spine_ik_snap_bake(RigifySpineIkSnapBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_spine_ik_snap_bake_" + rig_id
    bl_label = "Apply Snap To Hip IK To Keyframes"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the spine control keyframes corrected Hip IK result"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.master_bone, TRANSFORM_PROPS_LOCATION)
        return []
''']

def add_spine_ik_snap(panel, *, master=None, result=None, final=None, text=None):
    panel.use_bake_settings()
    panel.script.add_utilities(SCRIPT_UTILITIES_OP_SNAP)
    panel.script.register_classes(SCRIPT_REGISTER_OP_SNAP)

    op_props = {
        'master_bone': master, 'result_bone': result, 'final_bone': final,
    }

    row = panel.row(align=True)
    row.operator('pose.rigify_spine_ik_snap_{rig_id}', text=text, icon='SNAP_ON', properties=op_props)
    row.operator('pose.rigify_spine_ik_snap_bake_{rig_id}', text='', icon='ACTION_TWEAK', properties=op_props)
