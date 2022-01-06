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
import math
import json

from itertools import count
from mathutils import Vector

from rigify.utils.animation import add_generic_snap_fk_to_ik, add_fk_ik_snap_buttons
from rigify.utils.mechanism import driver_var_transform
from rigify.utils.bones import compute_chain_x_axis, align_bone_x_axis, set_bone_widget_transform
from rigify.utils.naming import make_derived_name
from rigify.utils.switch_parent import SwitchParentBuilder
from rigify.utils.layers import ControlLayersOption
from rigify.utils.misc import map_list
from rigify.utils.components import CustomPivotControl

from rigify.utils.widgets_basic import create_bone_widget

from rigify.base_rig import stage

from rigify.rigs.limbs import limb_rigs, leg


IK_MID_LAYERS = ControlLayersOption(
    'ik_mid', toggle_default=False,
    description="Layers for the mid joint IK controls to be on"
)


class BaseBodyIkLimbRig(limb_rigs.BaseLimbRig):
    ####################################################
    # BONES
    #
    # ctrl:
    #   ik_mid[]:
    #     Controls for the knee/elbow IK feature
    #   ik_mid_pivot (optional):
    #     Custom pivot for mid IK.
    # mch:
    #   ik_mid[]:
    #     Output of the knee/elbow IK.
    #   ik_mid_twist:
    #     Extra bone used if original bones aren't auto-aligned.
    #   ik_mid_pivot (optional):
    #     Custom pivot output for mid IK.
    #
    ####################################################

    ####################################################
    # Parent link

    def generate_bones(self):
        self.parent_org_bone = self.get_bone_parent(self.bones.org.main[0])

        # Replace the parent ORG bone of the limb with one provided by the Hip IK system
        parent = self.rigify_parent.get_body_ik_final_parent_bone()
        self.set_bone_parent(self.bones.org.main[0], parent)

        super().generate_bones()

        assert self.rig_parent_bone == parent

    @stage.parent_bones
    def parent_master_control(self):
        super().parent_master_control()

        # TODO: can't do this because of spurious dependency cycles
        #parent = self.rigify_parent.get_body_ik_safe_parent_bone()
        self.set_bone_parent(self.bones.ctrl.master, 'root', inherit_scale='NONE')

    ####################################################
    # UI

    def build_ik_parent_switch(self, pbuilder):
        super().build_ik_parent_switch(pbuilder)

        pbuilder.amend_child(self, self.bones.ctrl.ik, context_rig=self.rigify_parent)

    def register_switch_parents(self, pbuilder):
        parent = self.rig_parent_bone

        try:
            self.rig_parent_bone = self.parent_org_bone

            super().register_switch_parents(pbuilder)

        finally:
            self.rig_parent_bone = parent

    def add_global_buttons(self, panel, rig_name):
        super().add_global_buttons(panel, rig_name)

        self.make_property(
            self.prop_bone, 'IK_MID', default=0.0,
            description='{} IK Switch'.format(self.mid_control_name.title())
        )
        panel.custom_prop(
            self.prop_bone, 'IK_MID', slider=True,
            text='{} IK ({})'.format(self.mid_control_name.title(), rig_name)
        )

    def add_ik_only_buttons(self, panel, rig_name):
        super().add_ik_only_buttons(panel, rig_name)

        self.make_property(self.prop_bone, 'force_straight', 0.0, description='Force the IK limb straight')
        panel.custom_prop(self.prop_bone, 'force_straight', text='IK Force Straight', slider=True)

        ctrl = self.bones.ctrl
        mid_panel = self.script.panel_with_selected_check(self, [ctrl.master] + self.get_all_mid_ik_controls())

        self.add_mid_ik_only_buttons(mid_panel, rig_name)

    def add_mid_ik_only_buttons(self, panel, rig_name):
        ctrl = self.bones.ctrl
        cut = self.middle_ik_control_cutoff

        add_generic_snap_fk_to_ik(
            panel,
            fk_bones=ctrl.fk[0:cut],
            ik_bones=self.bones.mch.ik_mid,
            ik_ctrl_bones=ctrl.ik_mid,
            label='FK->{} IK'.format(self.mid_control_name.title()),
            rig_name=rig_name, compact=True,
        )

        add_limb_snap_mid_ik_to_fk(
            panel,
            master=ctrl.master,
            fk_bones=ctrl.fk[0:cut],
            ik_ctrl_bones=ctrl.ik_mid,
            ik_extra_ctrls=self.get_extra_mid_ik_controls(),
            label='{} IK->FK'.format(self.mid_control_name.title()),
            rig_name=rig_name, compact=True,
        )

        self.make_property(
            self.prop_bone, 'ik_mid_stretch', 1.0,
            description='Stretch factor for the first bone',
            min=0, max=10, soft_max=2
        )
        panel.custom_prop(
            self.prop_bone, 'ik_mid_stretch',
            text='{} IK Stretch'.format(self.mid_control_name.title())
        )

    ####################################################
    # Knee IK control

    middle_ik_control_cutoff = 3

    def get_all_mid_ik_controls(self):
        return self.bones.ctrl.ik_mid + self.get_extra_mid_ik_controls()

    def get_extra_mid_ik_controls(self):
        if self.component_mid_ik_pivot:
            return [self.component_mid_ik_pivot.control]
        else:
            return []

    def get_mid_ik_control_output(self):
        if self.component_mid_ik_pivot:
            return self.component_mid_ik_pivot.output
        else:
            return self.bones.ctrl.ik_mid[0]

    @stage.generate_bones
    def make_middle_ik_control_chain(self):
        cut = self.middle_ik_control_cutoff
        self.bones.ctrl.ik_mid = map_list(self.make_middle_ik_control_bone, count(0), self.bones.org.main[1:cut])

        ik_name = self.bones.ctrl.ik_mid[0]
        self.component_mid_ik_pivot = self.build_middle_ik_pivot(ik_name, scale=0.4)
        self.build_middle_ik_control_parent(ik_name)

    def build_middle_ik_pivot(self, ik_name, **args):
        if self.use_ik_pivot:
            return CustomPivotControl(self, 'ik_mid_pivot', ik_name, parent=ik_name, scale_mch=1, **args)

    def build_middle_ik_control_parent(self, ctrl):
        SwitchParentBuilder(self.generator).build_child(
            self, ctrl, context_rig=self.rigify_parent, select_parent='root',
            prop_bone=lambda:self.bones.ctrl.master,
            prop_id='IK_{}_parent'.format(self.mid_control_name),
            prop_name='IK {} Parent'.format(self.mid_control_name.title()),
            controls=lambda:self.get_all_mid_ik_controls() + [self.bones.ctrl.master],
        )

    def make_middle_ik_control_bone(self, i, org):
        return self.copy_bone(org, make_derived_name(org, 'ctrl', '_ik_mid'))

    @stage.parent_bones
    def parent_middle_ik_control_chain(self):
        ik_mid = self.bones.ctrl.ik_mid
        self.set_bone_parent(ik_mid[1], self.get_mid_ik_control_output(), use_connect=True)
        self.parent_bone_chain(ik_mid[1:], use_connect=False)

    @stage.configure_bones
    def configure_middle_ik_control_chain(self):
        IK_MID_LAYERS.assign(self.params, self.obj, self.get_all_mid_ik_controls())

    @stage.rig_bones
    def rig_middle_ik_control_chain(self):
        self.rig_ik_control_scale(self.bones.ctrl.ik_mid[0])

    @stage.generate_widgets
    def make_middle_ik_control_widgets(self):
        ctrls = self.bones.ctrl.ik_mid

        set_bone_widget_transform(self.obj, ctrls[0], self.get_mid_ik_control_output())

        for args in zip(count(0), ctrls):
            self.make_middle_ik_control_widget(*args)

    def make_middle_ik_control_widget(self, i, ctrl):
        if i == 0:
            create_bone_widget(self.obj, ctrl, r1=0.2, r2=0.1, l2=0.75)
        else:
            create_bone_widget(self.obj, ctrl, r1=0.3, r2=0.2, l1=0.2, l2=0.6)

    ####################################################
    # Knee IK MCH

    @stage.generate_bones
    def make_middle_ik_mch_chain(self):
        orgs = self.bones.org.main
        cut = self.middle_ik_control_cutoff
        self.bones.mch.ik_mid = map_list(self.make_middle_ik_mch_bone, count(0), orgs[0:cut])
        self.bones.mch.ik_mid_twist = self.make_mid_twist_mch_bone(orgs)

    def make_middle_ik_mch_bone(self, i, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '_ik_mid'))

    def make_mid_twist_mch_bone(self, orgs):
        if self.params.rotation_axis == 'automatic':
            return None

        name = self.copy_bone(orgs[0], make_derived_name(orgs[0], 'mch', '_ik_mid_twist'))
        axis = compute_chain_x_axis(self.obj, orgs[0:2])
        align_bone_x_axis(self.obj, name, axis)
        return name

    @stage.parent_bones
    def parent_middle_ik_mch_chain(self):
        mch = self.bones.mch

        if mch.ik_mid_twist:
            self.set_bone_parent(mch.ik_mid[0], mch.ik_mid_twist)
            self.set_bone_parent(mch.ik_mid_twist, mch.follow)
        else:
            self.set_bone_parent(mch.ik_mid[0], mch.follow)

        self.parent_bone_chain(mch.ik_mid[0:3], use_connect=True)
        self.parent_bone_chain(mch.ik_mid[2:], use_connect=False)

    @stage.rig_bones
    def rig_middle_ik_mch_chain(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl
        mid_ctrl = self.get_mid_ik_control_output()
        twist = mch.ik_mid_twist or mch.ik_mid[0]

        bone_twist = self.get_bone(twist)
        bone_ctrl = self.get_bone(mid_ctrl)

        if bone_twist.z_axis.dot(bone_ctrl.vector) > 0:
            axis = 'TRACK_Z'
        else:
            axis = 'TRACK_NEGATIVE_Z'

        self.make_constraint(twist, 'DAMPED_TRACK', mid_ctrl)
        self.make_constraint(
            twist, 'LOCKED_TRACK', mid_ctrl, head_tail=1,
            lock_axis='LOCK_Y', track_axis=axis,
        )

        for i in range(3):
            self.make_driver(
                mch.ik_mid[0], 'scale', index=i, variables=[(self.prop_bone, 'ik_mid_stretch')]
            )

        self.make_constraint(mch.ik_mid[1], 'DAMPED_TRACK', mid_ctrl, head_tail=1)
        self.make_constraint(mch.ik_mid[1], 'COPY_SCALE', mid_ctrl)

        self.make_constraint(mch.ik_mid[2], 'COPY_TRANSFORMS', ctrl.ik_mid[1])

        for mmch, mctrl in zip(mch.ik_mid[3:], ctrl.ik_mid[2:]):
            self.make_constraint(mmch, 'COPY_TRANSFORMS', mctrl, space='LOCAL')

    ####################################################
    # Knee IK output

    def rig_org_bone(self, i, org, fk, ik):
        super().rig_org_bone(i, org, fk, ik)

        ik_mid = self.bones.mch.ik_mid

        if i < len(ik_mid):
            con = self.make_constraint(org, 'COPY_TRANSFORMS', ik_mid[i])
            self.make_driver(con, 'influence', variables=[(self.prop_bone, 'IK_MID')])

    ####################################################
    # Body IK interface to the parent rig

    def get_snap_body_ik_controls(self):
        ctrl = self.bones.ctrl
        return [ *ctrl.ik_mid, ctrl.ik ]

    def rig_body_ik_target_bone(self, mch):
        "Called by the parent rig to rig the body ik target bone."

        lens = [ self.get_bone(name).length for name in self.bones.org.main[0:2] ]

        scale_root = driver_var_transform(self.obj, 'root', type='SCALE_AVG', space='LOCAL')
        scale_master = driver_var_transform(self.obj, self.bones.ctrl.master, type='SCALE_AVG', space='LOCAL')

        bone = self.get_bone(mch)
        bone['mode'] = 0.0

        self.make_driver(
            mch, '["mode"]', expression='1-k/min(1,f+k) if f > 0 else 0',
            variables={'f':(self.prop_bone, 'force_straight'), 'k':(self.prop_bone, 'IK_MID')}
        )
        self.make_driver(
            mch, '["influence"]', expression='min(1,f+k)',
            variables={'f':(self.prop_bone, 'force_straight'), 'k':(self.prop_bone, 'IK_MID')}
        )
        self.make_driver(
            mch, '["length"]',
            expression=f'rs*ms*lerp(t*{lens[0]},s*{sum(lens)},m)',
            variables={
                's': (self.bones.ctrl.ik_base, '.scale.y'),
                't': (self.prop_bone, 'ik_mid_stretch'),
                'rs': scale_root, 'ms': scale_master,
                'm': (mch, 'mode'),
            }
        )

        self.make_constraint(mch, 'COPY_LOCATION', self.get_mid_ik_control_output())

        con = self.make_constraint(
            mch, 'COPY_LOCATION', self.get_ik_input_bone(),
            head_tail=self.ik_input_head_tail
        )

        self.make_driver(con, 'influence', variables=[(mch, 'mode')])

    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        super().add_parameters(params)

        IK_MID_LAYERS.add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        super().parameters_ui(layout, params)

        IK_MID_LAYERS.parameters_ui(layout, params)


class BaseBodyIkLegRig(BaseBodyIkLimbRig):
    mid_control_name = 'knee'

    def initialize(self):
        from .spine_rigs import BaseBodyIkSpineRig

        super().initialize()

        if self.use_ik_toe:
            self.middle_ik_control_cutoff = 4

        if (not isinstance(self.rigify_parent, BaseBodyIkSpineRig) or
            self.get_bone_parent(self.bones.org.main[0]) != self.rigify_parent.bones.org[0]):
            self.raise_error('Hip IK leg must be a child of the IK spine hip bone.')

    ####################################################
    # FK parents MCH chain

    def rig_fk_parent_bone(self, i, parent_mch, org):
        super().rig_fk_parent_bone(i, parent_mch, org)

        if i == 3 and not self.use_ik_toe:
            self.make_driver(
                self.get_bone(parent_mch).constraints[0], 'influence', type='MAX',
                variables=[(self.prop_bone, 'IK_FK'), (self.prop_bone, 'IK_MID')],
                polynomial=[1.0, -1.0]
            )


class BaseBodyIkArmRig(BaseBodyIkLimbRig):
    mid_control_name = 'elbow'

    def initialize(self):
        from .shoulder import Rig as IkShoulderRig

        super().initialize()

        if not isinstance(self.rigify_parent, IkShoulderRig):
            self.raise_error('Body IK arm must be a child of the IK shoulder rig.')


#######################
# Snap knee IK to FK ##
#######################

SCRIPT_REGISTER_OP_SNAP_MID_IK_FK = ['POSE_OT_rigify_limb_mid_ik2fk', 'POSE_OT_rigify_limb_mid_ik2fk_bake']

SCRIPT_UTILITIES_OP_SNAP_MID_IK_FK = ['''
#######################
## Snap Mid IK to FK ##
#######################

class RigifyLimbMidIk2FkBase:
    prop_bone:      StringProperty(name="Master Control")
    stretch_prop:   StringProperty(name="Stretch Control", default='ik_mid_stretch')
    fk_bones:       StringProperty(name="FK Controls")
    ik_ctrl_bones:  StringProperty(name="IK Controls")
    ik_extra_ctrls: StringProperty(name="IK Extra Controls")

    keyflags = None

    def init_execute(self, context):
        self.fk_bone_list = json.loads(self.fk_bones)
        self.ik_bone_list = json.loads(self.ik_ctrl_bones)
        self.extra_ctrl_list = json.loads(self.ik_extra_ctrls)

    def save_frame_state(self, context, obj):
        return get_chain_transform_matrices(obj, self.fk_bone_list)

    def apply_frame_state(self, context, obj, matrices):
        mat = get_pose_matrix_in_other_space(matrices[0], obj.pose.bones[self.fk_bone_list[0]])
        set_custom_property_value(
            obj, self.prop_bone, self.stretch_prop, mat.to_scale()[1], keyflags=self.keyflags
        )

        # Remove extra control transform, if present
        for extra in self.extra_ctrl_list:
            set_transform_from_matrix(
                obj, extra, Matrix.Identity(4), space='LOCAL', keyflags=self.keyflags
            )

        set_chain_transforms_from_matrices(
            context, obj, self.ik_bone_list, matrices[1:], keyflags=self.keyflags,
            undo_copy_scale=True,
        )

class POSE_OT_rigify_limb_mid_ik2fk(RigifyLimbMidIk2FkBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_mid_ik2fk_" + rig_id
    bl_label = "Snap Mid IK->FK"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the middle joint IK control to FK"

class POSE_OT_rigify_limb_mid_ik2fk_bake(RigifyLimbMidIk2FkBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_mid_ik2fk_bake_" + rig_id
    bl_label = "Apply Snap Mid IK->FK To Keyframes"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Snap the middle joint IK control keyframes to FK"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.fk_bone_list, TRANSFORM_PROPS_ALL)
        ik_curves = self.bake_get_all_bone_curves(self.ik_bone_list + self.extra_ctrl_list, TRANSFORM_PROPS_ALL)
        prop_curves = self.bake_get_all_bone_custom_prop_curves(self.prop_bone, [self.stretch_prop])
        return ik_curves + prop_curves
''']

def add_limb_snap_mid_ik_to_fk(panel, *, master=None, fk_bones=[], ik_ctrl_bones=[], ik_extra_ctrls=[], label='IK->FK', rig_name='', compact=None):
    panel.use_bake_settings()
    panel.script.add_utilities(SCRIPT_UTILITIES_OP_SNAP_MID_IK_FK)
    panel.script.register_classes(SCRIPT_REGISTER_OP_SNAP_MID_IK_FK)

    op_props = {
        'prop_bone': master,
        'fk_bones': json.dumps(fk_bones),
        'ik_ctrl_bones': json.dumps(ik_ctrl_bones),
        'ik_extra_ctrls': json.dumps(ik_extra_ctrls),
    }

    add_fk_ik_snap_buttons(
        panel, 'pose.rigify_limb_mid_ik2fk_{rig_id}', 'pose.rigify_limb_mid_ik2fk_bake_{rig_id}',
        label=label, rig_name=rig_name, properties=op_props, clear_bones=ik_ctrl_bones, compact=compact,
    )
