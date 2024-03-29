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

import bpy
import math

from itertools import count
from mathutils import Matrix, Vector

from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import put_bone, align_bone_to_axis, set_bone_widget_transform, flip_bone
from rigify.utils.widgets import adjust_widget_transform_mesh
from rigify.utils.widgets_basic import create_circle_widget
from rigify.utils.misc import map_list
from rigify.utils.components import CustomPivotControl

from rigify.base_rig import stage

from rigify.rigs.spines.spine_rigs import BaseSpineRig


class Rig(BaseSpineRig):
    """
    BlenRig-like spine rig with movable hip and chest controls.
    """

    min_chain_length = 4

    chain_length: int
    use_hips_pivot: bool

    def initialize(self):
        super().initialize()

        self.chain_length = len(self.bones.org)

        self.use_hips_pivot = self.params.make_custom_hips_pivot

    ####################################################
    # BONES

    class CtrlBones(BaseSpineRig.CtrlBones):
        hips: str                      # Main hip control.
        chest: str                     # Main chest control
        ik_tweak: list[str]            # IK tweak controls
        tweak: list[str]               # Tweak control chain.

    class MchBones(BaseSpineRig.MchBones):
        ik_back: list[str]             # Inverted tracking chain
        ik_forward: list[str]          # Forward tracking chain

    bones: BaseSpineRig.ToplevelBones[
        list[str],
        'Rig.CtrlBones',
        'Rig.MchBones',
        list[str]
    ]

    ####################################################
    # Master control bone

    def get_master_control_pos(self, orgs):
        base_bone = self.get_bone(orgs[0])
        return (base_bone.head + base_bone.tail) / 2

    @stage.configure_bones
    def configure_master_control(self):
        super().configure_master_control()

        master = self.bones.ctrl.master
        self.make_property(
            master, 'fk_hips', 0.0,
            description='Hip bone rotation is defined purely by the main hips control')
        self.make_property(master, 'fk_chest', 0.0, description='Use FK controls for the chest')

        panel = self.script.panel_with_selected_check(self, self.bones.ctrl.flatten())
        panel.custom_prop(master, 'fk_hips', text='FK Hips', slider=True)
        panel.custom_prop(master, 'fk_chest', text='FK Chest', slider=True)

    ####################################################
    # Main control bones

    def get_end_control_pos(self, orgs: list[str], pos: float) -> Vector:
        pos *= len(orgs)
        idx = int(pos)
        base_bone = self.get_bone(orgs[idx])
        return base_bone.head + base_bone.vector * (pos - idx)

    component_hips_pivot: CustomPivotControl | None

    @stage.generate_bones
    def make_end_control_bones(self):
        orgs = self.bones.org

        self.bones.ctrl.hips = hips = self.make_hips_control_bone(orgs, 'hips')
        self.bones.ctrl.chest = self.make_chest_control_bone(orgs, 'chest')

        self.component_hips_pivot = self.build_hips_pivot(
            hips, position=self.get_end_control_pos(orgs, 1.5 / 4)
        )

    def make_hips_control_bone(self, orgs: list[str], name: str):
        name = self.copy_bone(orgs[0], name)
        put_bone(self.obj, name, self.get_bone(orgs[0]).tail)
        align_bone_to_axis(self.obj, name, 'z', length=self.length / 4)
        return name

    def make_chest_control_bone(self, orgs: list[str], name: str):
        name = self.copy_bone(orgs[-1], name)
        put_bone(self.obj, name, self.get_end_control_pos(orgs, 2.5 / 4))
        align_bone_to_axis(self.obj, name, 'z', length=self.length / 3)
        return name

    def build_hips_pivot(self, master_name: str, **args):
        if self.use_hips_pivot:
            return CustomPivotControl(
                self, 'hips_pivot', master_name, parent=master_name, **args
            )

    def get_hips_control_output(self):
        if self.component_hips_pivot:
            return self.component_hips_pivot.output
        else:
            return self.bones.ctrl.hips

    @stage.parent_bones
    def parent_end_control_bones(self):
        ctrl = self.bones.ctrl
        pivot = self.get_master_control_output()
        self.set_bone_parent(ctrl.hips, pivot)
        self.set_bone_parent(ctrl.chest, pivot)

    @stage.configure_bones
    def configure_end_control_bones(self):
        orgs = self.bones.org
        self.configure_end_control_bone(0, self.bones.ctrl.hips, orgs[0])
        self.configure_end_control_bone(1, self.bones.ctrl.chest, orgs[-1])

    def configure_end_control_bone(self, _i: int, ctrl: str, org: str):
        bone = self.get_bone(ctrl)
        bone.lock_scale = True, True, True
        bone.rotation_mode = self.get_bone(org).rotation_mode

    @stage.generate_widgets
    def make_end_control_widgets(self):
        ctrl = self.bones.ctrl
        ik_forward = self.bones.mch.ik_forward
        self.make_end_control_widget(ctrl.hips, ik_forward[0])
        self.make_end_control_widget(ctrl.chest, ik_forward[-2])

    def make_end_control_widget(self, ctrl: str, wgt_mch: str):
        shape_bone = self.get_bone(wgt_mch)
        is_horizontal = abs(shape_bone.z_axis.normalized().y) < 0.7

        set_bone_widget_transform(self.obj, ctrl, wgt_mch)

        obj = create_circle_widget(
            self.obj, ctrl,
            radius=1.2 if is_horizontal else 1.1,
            head_tail=0.0,
            head_tail_x=1.0,
            with_line=False,
        )

        if is_horizontal:
            # Tilt the widget toward the ground for horizontal (animal) spines
            angle = math.copysign(28, shape_bone.x_axis.x)
            rot_mat = Matrix.Rotation(math.radians(angle), 4, 'X')
            adjust_widget_transform_mesh(obj, rot_mat, local=True)

    ####################################################
    # IK tweak controls

    @stage.generate_bones
    def make_ik_tweak_chain(self):
        orgs = self.bones.org

        self.bones.ctrl.ik_tweak = [
            *map_list(self.make_ik_tweak_bone, count(1), orgs[1:]),
            self.make_ik_end_tweak_bone(orgs[-1])
        ]

    def make_ik_tweak_bone(self, _i: int, org: str):
        return self.copy_bone(org, make_derived_name(org, 'ctrl', '_ik'))

    def make_ik_end_tweak_bone(self, org: str):
        name = self.copy_bone(org, make_derived_name(org, 'ctrl', '_ik_end'), scale=0.5)
        put_bone(self.obj, name, self.get_bone(org).tail)
        return name

    @stage.parent_bones
    def parent_ik_tweak_chain(self):
        ik_tweak = self.bones.ctrl.ik_tweak

        pivot = self.get_master_control_output()
        for tweak in ik_tweak[:-2]:
            self.set_bone_parent(tweak, pivot)

        chest = self.bones.ctrl.chest
        for tweak in ik_tweak[-2:]:
            self.set_bone_parent(tweak, chest)

    @stage.configure_bones
    def configure_ik_tweak_chain(self):
        for args in zip(count(1), self.bones.ctrl.ik_tweak):
            self.configure_ik_tweak_bone(*args)

    def configure_ik_tweak_bone(self, _i: int, ctrl: str):
        bone = self.get_bone(ctrl)
        bone.lock_scale = True, True, True
        bone.rotation_mode = 'XZY'
        bone.lock_rotation = True, False, True

    @stage.rig_bones
    def rig_ik_tweak_chain(self):
        ctrl = self.bones.ctrl

        for i, tweak, in zip(count(1), ctrl.ik_tweak[:-2]):
            self.rig_ik_tweak_bone_mid(i, tweak, ctrl.hips, ctrl.chest)

        self.make_constraint(
            ctrl.ik_tweak[-2], 'COPY_ROTATION', ctrl.ik_tweak[-1],
            mix_mode='BEFORE', space='LOCAL'
        )

    def get_hips_weight(self, i: int):
        x = i * 4 / len(self.bones.org)
        return 1 + (1.0/24) * x - 0.375 * x * x + (1.0/12) * x * x * x

    def get_chest_weight(self, i: int):
        x = i * 4 / len(self.bones.org)
        return (47.0/60) * x - 0.5 * x * x + (7.0/60) * x * x * x

    def rig_ik_tweak_bone_mid(self, i: int, tweak: str, hips: str, chest: str):
        self.make_constraint(
            tweak, 'COPY_ROTATION', hips, mix_mode='ADD', space='LOCAL',
            use_xyz=(False, True, False),
            influence=self.get_hips_weight(i)
        )

        self.make_constraint(
            tweak, 'COPY_ROTATION', chest, mix_mode='ADD', space='LOCAL',
            use_xyz=(False, True, False),
            influence=self.get_chest_weight(i)
        )

    @stage.generate_widgets
    def generate_ik_tweak_widgets(self):
        for args in zip(count(1), self.bones.ctrl.ik_tweak, self.bones.mch.ik_forward[1:]):
            self.generate_ik_tweak_widget(*args)

    def generate_ik_tweak_widget(self, _i: int, ctrl: str, wgt_mch: str):
        set_bone_widget_transform(self.obj, ctrl, wgt_mch)

        obj = create_circle_widget(self.obj, ctrl, head_tail=0.0, with_line=False)

        adjust_widget_transform_mesh(obj, Matrix.Diagonal((1.1, 1, 0.9, 1)), local=True)

    ####################################################
    # FK control chain

    @stage.generate_bones
    def make_control_chain(self):
        self.bones.ctrl.fk = map_list(self.make_control_bone, count(0), self.bones.org[1:])

    @stage.parent_bones
    def parent_control_chain(self):
        self.set_bone_parent(self.bones.ctrl.fk[0], self.bones.ctrl.master)
        self.parent_bone_chain(self.bones.ctrl.fk, use_connect=True)

    @stage.configure_bones
    def configure_control_chain(self):
        for args in zip(count(0), self.bones.ctrl.fk, self.bones.org[1:]):
            self.configure_control_bone(*args)

        ControlLayersOption.FK.assign_rig(self, self.bones.ctrl.fk)

        shared_ctrls = [self.bones.ctrl.hips, self.bones.ctrl.master]
        if self.component_hips_pivot:
            shared_ctrls.append(self.component_hips_pivot.control)

        ControlLayersOption.FK.assign_rig(self, shared_ctrls, combine=True, priority=-1)

    @stage.configure_bones
    def rig_control_chain(self):
        for args in zip(count(0), self.bones.ctrl.fk, self.bones.mch.ik_forward[1:]):
            self.rig_control_bone(*args)

    def rig_control_bone(self, _i: int, fk: str, ik: str):
        con = self.make_constraint(fk, 'COPY_TRANSFORMS', ik)
        self.make_driver(con, 'influence',
                         variables=[(self.bones.ctrl.master, 'fk_chest')],
                         polynomial=[1.0, -1.0])

    def make_control_widget(self, i: int, ctrl: str):
        create_circle_widget(self.obj, ctrl, radius=1.0, head_tail=0.5)

    ####################################################
    # MCH IK back chain

    @stage.generate_bones
    def make_mch_ik_back_chain(self):
        self.bones.mch.ik_back = map_list(self.make_mch_ik_back_bone, count(1), self.bones.org[1:])

    def make_mch_ik_back_bone(self, _i: int, org: str):
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.ik_back'))
        flip_bone(self.obj, name)
        return name

    @stage.parent_bones
    def parent_mch_ik_back_chain(self):
        ik_back = self.bones.mch.ik_back

        self.set_bone_parent(ik_back[-1], self.bones.ctrl.ik_tweak[-1])
        self.parent_bone_chain(reversed(ik_back), use_connect=True)

    @stage.rig_bones
    def rig_mch_ik_back_chain(self):
        for args in zip(count(1), self.bones.mch.ik_back, self.bones.ctrl.ik_tweak):
            self.rig_mch_ik_back_bone(*args)

    def rig_mch_ik_back_bone(self, _i: int, back: str, tweak: str):
        self.make_constraint(back, 'DAMPED_TRACK', tweak)

    ####################################################
    # MCH IK forward chain

    @stage.generate_bones
    def make_mch_ik_forward_chain(self):
        self.bones.mch.ik_forward = [
            *map_list(self.make_mch_ik_forward_bone, count(0), self.bones.org),
            self.make_mch_ik_forward_end_bone(self.bones.org[-1])
        ]

    def make_mch_ik_forward_bone(self, _i: int, org: str):
        return self.copy_bone(org, make_derived_name(org, 'mch', '.ik_forward'))

    def make_mch_ik_forward_end_bone(self, org: str):
        name = self.copy_bone(org, make_derived_name(org, 'mch', '.ik_forward_end'), scale=0.5)
        put_bone(self.obj, name, self.get_bone(org).tail)
        return name

    @stage.parent_bones
    def parent_mch_ik_forward_chain(self):
        ik_forward = self.bones.mch.ik_forward

        self.set_bone_parent(ik_forward[0], self.get_hips_control_output())
        self.parent_bone_chain(ik_forward, use_connect=True)

    @stage.rig_bones
    def rig_mch_ik_forward_chain(self):
        ik_back = self.bones.mch.ik_back
        ik_tweak = self.bones.ctrl.ik_tweak

        for args in zip(count(0), self.bones.mch.ik_forward,
                        [*ik_back, ik_back[-1]], [None, *ik_tweak]):
            self.rig_mch_ik_forward_bone(*args)

    def rig_mch_ik_forward_bone(self, i: int, forward: str, back: str, tweak: str):
        if tweak:
            self.make_constraint(forward, 'COPY_ROTATION', tweak)

        if i == self.chain_length-2:
            self.make_constraint(
                forward, 'COPY_ROTATION', self.bones.ctrl.chest,
                mix_mode='ADD', space='LOCAL', use_xyz=(False, True, False),
                influence=1 - self.get_chest_weight(i)
            )

        trk_con = self.make_constraint(
            forward, 'DAMPED_TRACK', back,
            head_tail=0.0 if i == self.chain_length-1 else 1.0
        )

        if i == 0:
            self.make_driver(trk_con, 'influence',
                             variables=[(self.bones.ctrl.master, 'fk_hips')],
                             polynomial=[1.0, -1.0])

    ####################################################
    # Tweak bones

    @stage.parent_bones
    def parent_tweak_chain(self):
        fk = self.bones.ctrl.fk
        parents = [self.bones.mch.ik_forward[0], *fk, fk[-1]]
        for args in zip(self.bones.ctrl.tweak, parents):
            self.set_bone_parent(*args)

    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(cls, params):
        super().add_parameters(params)

        params.make_custom_hips_pivot = bpy.props.BoolProperty(
            name="Custom Hips Pivot Control",
            default=False,
            description="Create a rotation pivot control that can be repositioned "
                        "arbitrarily for the hips"
        )

        ControlLayersOption.FK.add_parameters(params)

    @classmethod
    def parameters_ui(cls, layout, params):
        super().parameters_ui(layout, params)

        layout.prop(params, 'make_custom_hips_pivot')

        ControlLayersOption.FK.parameters_ui(layout, params)


def create_sample(obj):
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('spine')
    bone.head = 0.0000, 0.0424, 1.0595
    bone.tail = 0.0000, 0.0173, 1.1569
    bone.roll = -0.0000
    bone.use_connect = False
    bones['spine'] = bone.name
    bone = arm.edit_bones.new('spine.001')
    bone.head = 0.0000, 0.0173, 1.1569
    bone.tail = 0.0000, 0.0019, 1.2778
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['spine']]
    bones['spine.001'] = bone.name
    bone = arm.edit_bones.new('spine.002')
    bone.head = 0.0000, 0.0019, 1.2778
    bone.tail = 0.0000, 0.0047, 1.4272
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['spine.001']]
    bones['spine.002'] = bone.name
    bone = arm.edit_bones.new('spine.003')
    bone.head = 0.0000, 0.0047, 1.4272
    bone.tail = 0.0000, 0.0114, 1.6582
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['spine.002']]
    bones['spine.003'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['spine']]
    pbone.rigify_type = 'spines.blenrig_spine'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'XZY'
    pbone = obj.pose.bones[bones['spine.001']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'XZY'
    pbone = obj.pose.bones[bones['spine.002']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'XZY'
    pbone = obj.pose.bones[bones['spine.003']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'XZY'

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
        if bcoll := arm.collections.active:
            bcoll.assign(bone)

    return bones
