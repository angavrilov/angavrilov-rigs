# ====================== BEGIN GPL LICENSE BLOCK ======================
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
# ======================= END GPL LICENSE BLOCK ========================

# <pep8 compliant>

from math import pi
from itertools import count, chain
from mathutils import Vector

from rigify.rig_ui_template import PanelLayout
from rigify.utils.naming import make_derived_name
from rigify.utils.misc import map_list
from rigify.utils.bones import put_bone
from rigify.utils.mechanism import driver_var_distance

from rigify.base_rig import stage

from rigify.rigs.spines import spine_rigs

from .limb_rigs import BaseBodyIkLimbParentRig, BaseBodyIkLegRig


class BaseBodyIkSpineRig(spine_rigs.BaseSpineRig, BaseBodyIkLimbParentRig):
    leg_rigs: list[BaseBodyIkLegRig]

    def initialize(self):
        super().initialize()

        orgs = self.bones.org

        legs = [
            child for child in self.rigify_children
            if (isinstance(child, BaseBodyIkLegRig) and
                self.get_bone_parent(child.bones.org.main[0]) == orgs[0])
        ]

        if len(legs) != 2:
            self.raise_error('IK spine requires two legs attached to the hip bone.')

        self.leg_rigs = legs

    ####################################################
    # BONES

    class MchBones(spine_rigs.BaseSpineRig.MchBones):
        hip_output: str                # Output of hip IK correction.
        hip_offset: str                # Global position represents the offset applied by Hip IK.
        tweak_offset: list[str]       # Offset bones used as parents for the tweak controls.
        leg_offset: list[str]          # Offset bones between leg base and hip base.
        hip_ik: list[str]              # Chain for solving the two point limit distance problem.
        hip_ik_tgt: str                # Target for two point IK

    bones: spine_rigs.BaseSpineRig.ToplevelBones[
        list[str],
        'BaseBodyIkSpineRig.CtrlBones',
        'BaseBodyIkSpineRig.MchBones',
        list[str]
    ]

    ####################################################
    # Master control bone

    @stage.configure_bones
    def configure_master_control(self):
        super().configure_master_control()

        leg_controls = list(chain.from_iterable(
            leg.get_snap_body_ik_controls() for leg in self.leg_rigs))

        panel = self.script.panel_with_selected_check(
            self, self.bones.ctrl.flatten() + leg_controls)

        self.generate_body_ik_panel(panel)

    def generate_body_ik_panel(self, panel: PanelLayout):
        add_spine_ik_snap(
            panel,
            master=self.bones.ctrl.master,
            result=self.get_pre_hip_ik_result_bone(),
            final=self.bones.org[0]
        )

    def get_pre_hip_ik_result_bone(self):
        raise NotImplementedError

    ####################################################
    # Leg offset MCH bones

    @stage.generate_bones
    def make_leg_offset_mch_bones(self):
        self.bones.mch.leg_offset = map_list(
            self.make_leg_offset_mch_bone, count(0), self.leg_rigs)

    def make_leg_offset_mch_bone(self, _i: int, leg_rig: BaseBodyIkLegRig):
        org = leg_rig.bones.org.main[0]
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik'))
        self.get_bone(name).tail = self.get_bone(self.bones.org[0]).head
        return name

    @stage.parent_bones
    def parent_leg_offset_mch_bones(self):
        for mch in self.bones.mch.leg_offset:
            self.set_bone_parent(mch, self.get_pre_hip_ik_result_bone())

    @stage.configure_bones
    def configure_leg_offset_mch_bones(self):
        for args in zip(count(0), self.leg_rigs, self.bones.mch.leg_offset):
            self.configure_leg_offset_mch_bone(*args)

    def configure_leg_offset_mch_bone(self, _i: int, _leg_rig: BaseBodyIkLegRig, mch: str):
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
        self.bones.mch.hip_ik_tgt = self.copy_bone(
            org, make_derived_name(org, 'mch', '.hip_ik_tgt'), scale=1/5)
        mch1 = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik'))
        mch2 = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_ik_end'))
        bone1 = self.get_bone(mch1)
        bone2 = self.get_bone(mch2)
        bone1.tail = bone2.head = bone1.head + Vector((0, 0, 1))
        bone2.tail = bone1.head + Vector((1, 0, 1))
        bone1.roll = bone2.roll = 0
        bone2.use_inherit_scale = False
        self.bones.mch.hip_ik = [mch1, mch2]

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
        self.rig_hip_ik_system(
            mch.hip_ik[0], mch.hip_ik[1], mch.hip_ik_tgt, self.get_pre_hip_ik_result_bone())

    def rig_hip_ik_mch_bone(self, _i: int, mch_ik: str, mch_in: str):
        self.make_driver(mch_ik, 'scale', index=1, variables=[(mch_in, 'length')])

    def rig_hip_ik_system(self, _mch_base: str, mch_ik: str, mch_tgt: str, pole: str):
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

    def rig_hip_ik_output(self, out: str, lim_both: str, lim_in1: str, lim_in2: str,
                          dist1: str, dist2: str):
        inf_vars = {
            'inf1': (lim_in1, 'influence'),
            'inf2': (lim_in2, 'influence'),
            'dist': driver_var_distance(self.obj, bone1=dist1, bone2=dist2),
        }

        scale = self.get_bone(lim_both).length * 0.001

        step_out = f'smoothstep({scale:.1e},{scale*2:.1e},dist)'
        step_in = f'smoothstep({-scale*3:.1e},{-scale*2:.1e},-dist)'

        con = self.make_constraint(out, 'COPY_LOCATION', lim_both)
        self.make_driver(
            con, 'influence', variables=inf_vars, expression=f'min(inf1,inf2)*{step_out}')

        con = self.make_constraint(
            out, 'LIMIT_DISTANCE', lim_in1, head_tail=1, space='POSE',
            limit_mode='LIMITDIST_ONSURFACE', distance=1,
        )
        self.make_driver(con, 'distance', variables=[(lim_in1, 'length')])
        self.make_driver(
            con, 'influence', variables=inf_vars,
            expression=f'lerp(min(inf1,inf2)*{step_in},1,(inf1-inf2)/(1-inf2) if inf1 > inf2 else 0)'  # noqa: E501
        )

        con = self.make_constraint(
            out, 'LIMIT_DISTANCE', lim_in2, head_tail=1, space='POSE',
            limit_mode='LIMITDIST_ONSURFACE', distance=1,
        )
        self.make_driver(con, 'distance', variables=[(lim_in2, 'length')])
        self.make_driver(
            con, 'influence', variables=inf_vars,
            expression=f'lerp(min(inf1,inf2)*{step_in},1,(inf2-inf1)/(1-inf1) if inf2 > inf1 else 0)'  # noqa: E501
        )

    ####################################################
    # Hip offset bones

    @stage.generate_bones
    def make_hip_offset_bones(self):
        org = self.bones.org
        mch = self.bones.mch
        mch.hip_output = self.copy_bone(
            org[0], make_derived_name(org[0], 'mch', '.hip_output'), scale=0.25)
        mch.hip_offset = self.copy_bone(
            org[0], make_derived_name(org[0], 'mch', '.hip_offset'), scale=0.20)

    @stage.parent_bones
    def parent_hip_offset_bones(self):
        mch = self.bones.mch
        self.set_bone_parent(mch.hip_output, self.get_pre_hip_ik_result_bone())
        self.set_bone_parent(mch.hip_offset, mch.hip_output)

    @stage.rig_bones
    def rig_hip_offset_bones(self):
        mch = self.bones.mch

        self.rig_hip_ik_output(
            mch.hip_output, mch.hip_ik[1], mch.leg_offset[0], mch.leg_offset[1],
            mch.hip_ik[0], mch.hip_ik_tgt)
        self.make_constraint(mch.hip_offset, 'COPY_LOCATION', self.get_hip_offset_base_bone(),
                             invert_xyz=(True, True, True), use_offset=True, space='POSE')

    def get_hip_offset_base_bone(self):
        return self.get_pre_hip_ik_result_bone()

    def get_body_ik_safe_parent_bone(self):
        """Parent bone for Body IK child limbs that doesn't depend on the IK"""
        return self.get_pre_hip_ik_result_bone()

    def get_body_ik_final_parent_bone(self):
        """Parent bone for Body IK child limbs that does depend on the IK"""
        return self.bones.mch.hip_output

    ####################################################
    # Apply offsets to tweaks

    def get_first_tweak_parent(self):
        return self.bones.mch.hip_output

    @stage.generate_bones
    def make_tweak_offset_chain(self):
        orgs = self.bones.org

        # It's necessary to inject offsets into tweak parents for connected
        # head/tail to work correctly. Trying to optimize by applying offsets
        # to ORG bones doesn't work.
        self.bones.mch.tweak_offset = map_list(
            self.make_tweak_offset_bone, count(1), orgs[1:] + orgs[-1:])

    def make_tweak_offset_bone(self, i: int, org: str):
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.tweak_offset'),
                              parent=False, scale=0.25)

        if i == len(self.bones.org):
            put_bone(self.obj, name, self.get_bone(org).tail)

        return name

    @stage.parent_bones
    def parent_tweak_chain(self):
        super().parent_tweak_chain()

        tweak = self.bones.ctrl.tweak

        self.set_bone_parent(tweak[0], self.get_first_tweak_parent())

        for i, tweak, offset in zip(count(1), tweak[1:], self.bones.mch.tweak_offset):
            self.parent_tweak_offset_bone(i, tweak, offset, self.get_bone_parent(tweak))

    def parent_tweak_offset_bone(self, _i: int, tweak: str, offset: str, tweak_parent: str):
        self.set_bone_parent(tweak, offset)
        self.set_bone_parent(offset, tweak_parent)

    @stage.rig_bones
    def rig_tweak_offset_chain(self):
        for i, offset in zip(count(1), self.bones.mch.tweak_offset):
            self.rig_tweak_offset_bone(i, offset)

    def rig_tweak_offset_bone(self, _i: int, offset: str):
        self.make_constraint(
            offset, 'COPY_LOCATION', self.bones.mch.hip_offset,
            use_offset=True, space='POSE'
        )


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

        set_transform_from_matrix(
            obj, self.master_bone, matrix, no_rot=True, no_scale=True, keyflags=self.keyflags)

class POSE_OT_rigify_spine_ik_snap(
        RigifySpineIkSnapBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_spine_ik_snap_" + rig_id
    bl_label = "Snap To Hip IK"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the spine control to corrected Hip IK result"

class POSE_OT_rigify_spine_ik_snap_bake(
        RigifySpineIkSnapBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_spine_ik_snap_bake_" + rig_id
    bl_label = "Apply Snap To Hip IK To Keyframes"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the spine control keyframes corrected Hip IK result"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.master_bone, TRANSFORM_PROPS_LOCATION)
        return []
''']


def add_spine_ik_snap(panel: PanelLayout, *, master=None, result=None, final=None, text=None):
    panel.use_bake_settings()
    panel.script.add_utilities(SCRIPT_UTILITIES_OP_SNAP)
    panel.script.register_classes(SCRIPT_REGISTER_OP_SNAP)

    op_props = {
        'master_bone': master, 'result_bone': result, 'final_bone': final,
    }

    row = panel.row(align=True)
    row.operator('pose.rigify_spine_ik_snap_{rig_id}',
                 text=text, icon='SNAP_ON', properties=op_props)
    row.operator('pose.rigify_spine_ik_snap_bake_{rig_id}',
                 text='', icon='ACTION_TWEAK', properties=op_props)
