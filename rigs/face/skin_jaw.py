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

from rigify.utils.naming import make_derived_name, SideZ, get_name_side_z
from rigify.utils.bones import align_bone_z_axis, put_bone
from rigify.utils.misc import map_list, matrix_from_axis_pair

from rigify.rigs.widgets import create_jaw_widget

from rigify.base_rig import stage, RigComponent

from ..skin.skin_rigs import BaseSkinRig, ControlBoneNode, ControlBoneParentOrg, LazyRef
from ..skin.basic_chain import Rig as BasicChainRig

import mathutils

from itertools import count
from mathutils import Vector, Matrix


class Rig(BaseSkinRig):
    """Jaw rig."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()


    ####################################################
    # Control nodes

    def get_parent_for_name(self, name, parent_bone):
        if parent_bone == self.base_bone:
            side = get_name_side_z(name)
            if side == SideZ.TOP:
                return LazyRef(self.bones.mch, 'top')
            if side == SideZ.BOTTOM:
                return LazyRef(self.bones.mch, 'bottom')

        return parent_bone

    def get_child_chain_parent(self, rig, parent_bone):
        return self.get_parent_for_name(rig.base_bone, parent_bone)

    def build_control_node_parent(self, node, parent_bone):
        return ControlBoneParentOrg(self.get_parent_for_name(node.name, parent_bone))


    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control(self):
        org = self.bones.org
        name = self.copy_bone(org, make_derived_name(org, 'ctrl'), parent=True)
        self.bones.ctrl.master = name

    @stage.configure_bones
    def configure_master_control(self):
        self.copy_bone_properties(self.bones.org, self.bones.ctrl.master)

    @stage.generate_widgets
    def make_master_control_widget(self):
        ctrl = self.bones.ctrl.master
        create_jaw_widget(self.obj, ctrl)


    ####################################################
    # Tracking MCH

    @stage.generate_bones
    def make_mch_lock_bones(self):
        org = self.bones.org
        mch = self.bones.mch

        mch.lock = self.copy_bone(org, make_derived_name(org, 'mch', '_lock'), scale=1/2, parent=True)
        mch.top = self.copy_bone(org, make_derived_name(org, 'mch', '_top'), scale=1/4, parent=True)
        mch.bottom = self.copy_bone(org, make_derived_name(org, 'mch', '_bottom'), scale=1/3, parent=True)

    @stage.configure_bones
    def configure_mch_lock_bones(self):
        ctrl = self.bones.ctrl

        panel = self.script.panel_with_selected_check(self, [ctrl.master])

        self.make_property(ctrl.master, 'mouth_lock', 0.0, description='Mouth is locked closed')
        panel.custom_prop(ctrl.master, 'mouth_lock', text='Mouth Lock', slider=True)

    @stage.rig_bones
    def rig_mch_track_bones(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl

        self.make_constraint(
            mch.lock, 'COPY_TRANSFORMS', ctrl.master,
            influence=self.params.jaw_locked_influence,
        )

        con = self.make_constraint(mch.top, 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])

        self.make_constraint(
            mch.bottom, 'COPY_TRANSFORMS', ctrl.master,
            influence=self.params.jaw_mouth_influence,
        )

        con = self.make_constraint(mch.bottom, 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])


    ####################################################
    # ORG bone

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org, self.bones.ctrl.master, inherit_scale='FULL')


    ####################################################
    # Deform bones

    @stage.generate_bones
    def make_deform_bone(self):
        org = self.bones.org
        deform = self.bones.deform
        self.bones.deform.master = self.copy_bone(org, make_derived_name(org, 'def'))

    @stage.parent_bones
    def parent_deform_chain(self):
        deform = self.bones.deform
        self.set_bone_parent(deform.master, self.bones.org)


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.jaw_mouth_influence = bpy.props.FloatProperty(
            name = "Bottom Lip Influence",
            default = 0.5, min = 0, max = 1,
            description = "Influence of the jaw on the bottom lip chains"
        )

        params.jaw_locked_influence = bpy.props.FloatProperty(
            name = "Locked Influence",
            default = 0.2, min = 0, max = 1,
            description = "Influence of the jaw on the locked mouth"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "jaw_mouth_influence", slider=True)
        layout.prop(params, "jaw_locked_influence", slider=True)


def create_sample(obj):
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('jaw')
    bone.head = 0.0000, 0.0000, 0.0000
    bone.tail = 0.0000, -0.0585, -0.0489
    bone.roll = 0.0000
    bone.use_connect = False
    bones['jaw'] = bone.name
    bone = arm.edit_bones.new('teeth.T')
    bone.head = 0.0000, -0.0589, 0.0080
    bone.tail = 0.0000, -0.0283, 0.0080
    bone.roll = 0.0000
    bone.use_connect = False
    bones['teeth.T'] = bone.name
    bone = arm.edit_bones.new('lip.T.L')
    bone.head = -0.0000, -0.0684, 0.0030
    bone.tail = 0.0105, -0.0655, 0.0033
    bone.roll = -0.0000
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['jaw']]
    bones['lip.T.L'] = bone.name
    bone = arm.edit_bones.new('lip.B.L')
    bone.head = -0.0000, -0.0655, -0.0078
    bone.tail = 0.0107, -0.0625, -0.0053
    bone.roll = -0.0551
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['jaw']]
    bones['lip.B.L'] = bone.name
    bone = arm.edit_bones.new('lip.T.R')
    bone.head = 0.0000, -0.0684, 0.0030
    bone.tail = -0.0105, -0.0655, 0.0033
    bone.roll = 0.0000
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['jaw']]
    bones['lip.T.R'] = bone.name
    bone = arm.edit_bones.new('lip.B.R')
    bone.head = 0.0000, -0.0655, -0.0078
    bone.tail = -0.0107, -0.0625, -0.0053
    bone.roll = 0.0551
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['jaw']]
    bones['lip.B.R'] = bone.name
    bone = arm.edit_bones.new('teeth.B')
    bone.head = 0.0000, -0.0543, -0.0136
    bone.tail = 0.0000, -0.0237, -0.0136
    bone.roll = 0.0000
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['jaw']]
    bones['teeth.B'] = bone.name
    bone = arm.edit_bones.new('lip1.T.L')
    bone.head = 0.0105, -0.0655, 0.0033
    bone.tail = 0.0193, -0.0586, 0.0007
    bone.roll = -0.0257
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip.T.L']]
    bones['lip1.T.L'] = bone.name
    bone = arm.edit_bones.new('lip1.B.L')
    bone.head = 0.0107, -0.0625, -0.0053
    bone.tail = 0.0194, -0.0573, -0.0029
    bone.roll = 0.0716
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip.B.L']]
    bones['lip1.B.L'] = bone.name
    bone = arm.edit_bones.new('lip1.T.R')
    bone.head = -0.0105, -0.0655, 0.0033
    bone.tail = -0.0193, -0.0586, 0.0007
    bone.roll = 0.0257
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip.T.R']]
    bones['lip1.T.R'] = bone.name
    bone = arm.edit_bones.new('lip1.B.R')
    bone.head = -0.0107, -0.0625, -0.0053
    bone.tail = -0.0194, -0.0573, -0.0029
    bone.roll = -0.0716
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip.B.R']]
    bones['lip1.B.R'] = bone.name
    bone = arm.edit_bones.new('lip2.T.L')
    bone.head = 0.0193, -0.0586, 0.0007
    bone.tail = 0.0236, -0.0539, -0.0014
    bone.roll = 0.0324
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip1.T.L']]
    bones['lip2.T.L'] = bone.name
    bone = arm.edit_bones.new('lip2.B.L')
    bone.head = 0.0194, -0.0573, -0.0029
    bone.tail = 0.0236, -0.0539, -0.0014
    bone.roll = 0.0467
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip1.B.L']]
    bones['lip2.B.L'] = bone.name
    bone = arm.edit_bones.new('lip2.T.R')
    bone.head = -0.0193, -0.0586, 0.0007
    bone.tail = -0.0236, -0.0539, -0.0014
    bone.roll = -0.0324
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip1.T.R']]
    bones['lip2.T.R'] = bone.name
    bone = arm.edit_bones.new('lip2.B.R')
    bone.head = -0.0194, -0.0573, -0.0029
    bone.tail = -0.0236, -0.0539, -0.0014
    bone.roll = -0.0467
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['lip1.B.R']]
    bones['lip2.B.R'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['jaw']]
    pbone.rigify_type = 'face.skin_jaw'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['teeth.T']]
    pbone.rigify_type = 'basic.super_copy'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.make_deform = False
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.super_copy_widget_type = "teeth"
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['lip.T.L']]
    pbone.rigify_type = 'skin.stretchy_chain'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.bbones = 3
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff_spherical = [True, False, True]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff = [0.5, 1.0, -0.5]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_connect_mirror = [True, False]
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['lip.B.L']]
    pbone.rigify_type = 'skin.stretchy_chain'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.bbones = 3
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff_spherical = [True, False, True]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff = [0.5, 1.0, -0.5]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_connect_mirror = [True, False]
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['lip.T.R']]
    pbone.rigify_type = 'skin.stretchy_chain'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.bbones = 3
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff_spherical = [True, False, True]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff = [0.5, 1.0, -0.5]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_connect_mirror = [True, False]
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['lip.B.R']]
    pbone.rigify_type = 'skin.stretchy_chain'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.bbones = 3
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff_spherical = [True, False, True]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_falloff = [0.5, 1.0, -0.5]
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.skin_chain_connect_mirror = [True, False]
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['teeth.B']]
    pbone.rigify_type = 'basic.super_copy'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    try:
        pbone.rigify_parameters.super_copy_widget_type = "teeth"
    except AttributeError:
        pass
    try:
        pbone.rigify_parameters.make_deform = False
    except AttributeError:
        pass
    pbone = obj.pose.bones[bones['lip1.T.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip1.B.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip1.T.R']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip1.B.R']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip2.T.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip2.B.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip2.T.R']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['lip2.B.R']]
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
        bone.bbone_x = bone.bbone_z = bone.length * 0.05
        arm.edit_bones.active = bone

    return bones
