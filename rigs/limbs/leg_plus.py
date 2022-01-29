import bpy

import re
import itertools
import bisect
import math
import json

from mathutils import Vector

from rigify.utils.animation import add_fk_ik_snap_buttons
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import put_bone
from rigify.utils.mechanism import driver_var_transform
from rigify.utils.misc import map_list, map_apply

from rigify.base_rig import stage
from itertools import count, repeat

from rigify.rigs.limbs import leg, limb_rigs


DEG_360 = math.pi * 2


class Rig(leg.Rig):
    """Human leg rig extensions."""

    def initialize(self):
        super().initialize()

        self.use_toe_roll = self.params.extra_toe_roll

    def add_global_buttons(self, panel, rig_name):
        super().add_global_buttons(panel, rig_name)

        ik_chain, tail_chain, fk_chain = self.get_ik_fk_position_chains()

        add_leg_snap_ik_to_fk(
            panel,
            master=self.bones.ctrl.master,
            fk_bones=fk_chain, ik_bones=ik_chain, tail_bones=tail_chain,
            ik_ctrl_bones=self.get_ik_control_chain(),
            ik_extra_ctrls=self.get_extra_ik_controls(),
            heel_control=self.bones.ctrl.heel,
            rig_name=rig_name
        )

    def add_ik_only_buttons(self, panel, rig_name):
        super().add_ik_only_buttons(panel, rig_name)

        if self.use_toe_roll:
            bone = self.bones.ctrl.heel

            self.make_property(
                bone, 'Toe_Roll', default=0.0,
                description='Roll forward from the tip of the toe'
            )

            panel.custom_prop(bone, 'Toe_Roll', text='Roll Forward On Toe', slider=True)

    def make_roll_mch_bones(self, foot, toe, heel):
        chain = super().make_roll_mch_bones(foot, toe, heel)

        if self.use_toe_roll:
            rock2, rock1, roll2, roll1, result = chain

            roll3 = self.copy_bone(toe, make_derived_name(heel, 'mch', '_roll3'), scale=0.3)

            toe_pos = Vector(self.get_bone(toe).tail)
            toe_pos.z = self.get_bone(roll2).head.z

            put_bone(self.obj, roll3, toe_pos, matrix=self.roll_matrix)

            return [rock2, rock1, roll2, roll3, roll1, result]

        return chain

    def rig_roll_mch_bones(self, chain, heel, org_heel):
        if self.use_toe_roll:
            rock2, rock1, roll2, roll3, roll1, result = chain

            bone = self.get_bone(roll3)
            bone.rotation_mode = self.heel_euler_order

            # Interpolate rotation in Euler space via drivers to simplify Snap With Roll
            self.make_driver(
                bone, 'rotation_euler', index=0,
                expression='max(0,x*i)' if self.main_axis == 'x' else 'x*i',
                variables={
                    'x': driver_var_transform(
                        self.obj, heel, type='ROT_X', space='LOCAL',
                        rotation_mode=self.heel_euler_order,
                    ),
                    'i': (heel, 'Toe_Roll'),
                }
            )

            self.make_driver(
                bone, 'rotation_euler', index=2,
                expression='max(0,z*i)' if self.main_axis == 'z' else 'z*i',
                variables={
                    'z': driver_var_transform(
                        self.obj, heel, type='ROT_Z', space='LOCAL',
                        rotation_mode=self.heel_euler_order,
                    ),
                    'i': (heel, 'Toe_Roll'),
                }
            )

            chain = [rock2, rock1, roll2, roll1, result]

        super().rig_roll_mch_bones(chain, heel, org_heel)

    ####################################################
    # Settings

    @classmethod
    def add_parameters(self, params):
        super().add_parameters(params)

        params.extra_toe_roll = bpy.props.BoolProperty(
            name='Toe Tip Roll',
            default=False,
            description="Generate a slider to pivot forward roll from the tip of the toe"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, 'extra_toe_roll')

        super().parameters_ui(layout, params)


def create_sample(obj):
    bones = leg.create_sample(obj)
    pbone = obj.pose.bones[bones['thigh.L']]
    pbone.rigify_type = 'limbs.leg_plus'


##########################
# Leg IK to FK operator ##
##########################

SCRIPT_REGISTER_OP_SNAP_IK_FK = [
    'POSE_OT_rigify_leg_roll_ik2fk', 'POSE_OT_rigify_leg_roll_ik2fk_bake']

SCRIPT_UTILITIES_OP_SNAP_IK_FK = limb_rigs.SCRIPT_UTILITIES_OP_SNAP_IK_FK + ['''
#######################
## Leg Snap IK to FK ##
#######################

class RigifyLegRollIk2FkBase(RigifyLimbIk2FkBase):
    heel_control: StringProperty(name="Heel")
    use_roll:     bpy.props.BoolVectorProperty(
        name="Use Roll", size=3, default=(True, True, False),
        description="Specifies which rotation axes of the heel roll control to use"
    )

    MODES = {
        'ZXY': ((0, 2), (1, 0, 2)),
        'XZY': ((2, 0), (2, 0, 1)),
    }

    def save_frame_state(self, context, obj):
        return get_chain_transform_matrices(obj, self.fk_bone_list + self.ctrl_bone_list[-1:])

    def assign_extra_controls(self, context, obj, all_matrices, ik_bones, ctrl_bones):
        for extra in self.extra_ctrl_list:
            set_transform_from_matrix(
                obj, extra, Matrix.Identity(4), space='LOCAL', keyflags=self.keyflags
            )

        if any(self.use_roll):
            foot_matrix = all_matrices[len(ik_bones) - 1]
            ctrl_matrix = all_matrices[len(self.fk_bone_list)]
            heel_bone = obj.pose.bones[self.heel_control]
            foot_bone = ctrl_bones[-1]

            # Relative rotation of heel from orientation of master IK control to actual foot orientation
            heel_rest = convert_pose_matrix_via_rest_delta(ctrl_matrix, foot_bone, heel_bone)
            heel_rot = convert_pose_matrix_via_rest_delta(foot_matrix, ik_bones[-1], heel_bone)

            # Decode the euler decomposition mode
            rotmode = heel_bone.rotation_mode
            indices, usemap = self.MODES[rotmode]
            use_roll = [ self.use_roll[i] for i in usemap ]
            roll, turn = indices

            # If the last rotation (yaw) is unused, move it to be first for better result
            if not use_roll[turn]:
                rotmode = rotmode[1:] + rotmode[0:1]

            local_rot = (heel_rest.inverted() @ heel_rot).to_euler(rotmode)

            heel_bone.rotation_euler = [ (val if use else 0) for val, use in zip(local_rot, use_roll) ]

            if self.keyflags is not None:
                keyframe_transform_properties(
                    obj, bone_name, self.keyflags, no_loc=True, no_rot=no_rot, no_scale=True
                )

            if 'Toe_Roll' in heel_bone and self.tail_bone_list:
                toe_matrix = all_matrices[len(ik_bones)]
                toe_bone = obj.pose.bones[self.tail_bone_list[0]]

                # Compute relative rotation of heel determined by toe
                heel_rot_toe = convert_pose_matrix_via_rest_delta(toe_matrix, toe_bone, heel_bone)
                toe_rot = (heel_rest.inverted() @ heel_rot_toe).to_euler(rotmode)

                # Determine how much of the already computed heel rotation seems to be applied
                heel_rot = list(heel_bone.rotation_euler)
                heel_rot[roll] = max(0.0, heel_rot[roll])

                # This relies on toe roll interpolation being done in Euler space
                ratios = [
                    toe_rot[i] / heel_rot[i] for i in (roll, turn)
                    if use_roll[i] and heel_rot[i] * toe_rot[i] > 0
                ]

                val = min(1.0, max(0.0, min(ratios) if ratios else 0.0))
                if val < 1e-5:
                    val = 0.0

                set_custom_property_value(obj, heel_bone.name, 'Toe_Roll', val, keyflags=self.keyflags)

    def draw(self, context):
        row = self.layout.row(align=True)
        row.label(text="Use:")
        row.prop(self, 'use_roll', index=0, text="Rock", toggle=True)
        row.prop(self, 'use_roll', index=1, text="Roll", toggle=True)
        row.prop(self, 'use_roll', index=2, text="Yaw", toggle=True)

class POSE_OT_rigify_leg_roll_ik2fk(RigifyLegRollIk2FkBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_options = {'REGISTER', 'UNDO', 'INTERNAL'}
    bl_idname = "pose.rigify_leg_roll_ik2fk_" + rig_id
    bl_label = "Snap IK->FK With Roll"
    bl_description = "Snap the IK chain to FK result, using foot roll to preserve the current IK control orientation as much as possible"

    def invoke(self, context, event):
        self.init_invoke(context)
        return self.execute(context)

class POSE_OT_rigify_leg_roll_ik2fk_bake(RigifyLegRollIk2FkBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_leg_roll_ik2fk_bake_" + rig_id
    bl_label = "Apply Snap IK->FK To Keyframes"
    bl_description = "Snap the IK chain keyframes to FK result, using foot roll to preserve the current IK control orientation  as much as possible"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.fk_bone_list, TRANSFORM_PROPS_ALL)
        self.bake_add_bone_frames(self.ctrl_bone_list[-1:], TRANSFORM_PROPS_ROTATION)
        return self.bake_get_all_bone_curves(self.ctrl_bone_list + self.extra_ctrl_list, TRANSFORM_PROPS_ALL)
''']


def add_leg_snap_ik_to_fk(panel, *, master=None, fk_bones=[], ik_bones=[], tail_bones=[], ik_ctrl_bones=[], ik_extra_ctrls=[], heel_control, rig_name=''):
    panel.use_bake_settings()
    panel.script.add_utilities(SCRIPT_UTILITIES_OP_SNAP_IK_FK)
    panel.script.register_classes(SCRIPT_REGISTER_OP_SNAP_IK_FK)

    assert len(fk_bones) == len(ik_bones) + len(tail_bones)

    op_props = {
        'prop_bone': master,
        'fk_bones': json.dumps(fk_bones),
        'ik_bones': json.dumps(ik_bones),
        'ctrl_bones': json.dumps(ik_ctrl_bones),
        'tail_bones': json.dumps(tail_bones),
        'extra_ctrls': json.dumps(ik_extra_ctrls),
        'heel_control': heel_control,
    }

    add_fk_ik_snap_buttons(
        panel, 'pose.rigify_leg_roll_ik2fk_{rig_id}', 'pose.rigify_leg_roll_ik2fk_bake_{rig_id}',
        label='IK->FK With Roll', rig_name=rig_name, properties=op_props,
    )
