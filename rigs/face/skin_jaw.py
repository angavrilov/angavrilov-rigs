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

from rigify.utils.naming import make_derived_name, Side, SideZ, get_name_side_z
from rigify.utils.bones import align_bone_z_axis, put_bone
from rigify.utils.misc import map_list, matrix_from_axis_pair

from rigify.rigs.widgets import create_jaw_widget

from rigify.base_rig import stage, RigComponent

from ..skin.skin_rigs import BaseSkinRig, ControlBoneNode, ControlBoneParentOrg, ControlBoneParentArmature, LazyRef
from ..skin.basic_chain import Rig as BasicChainRig

import mathutils

from itertools import count, repeat
from mathutils import Vector, Matrix
from bl_math import clamp


class Rig(BaseSkinRig):
    """Jaw rig."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.mouth_orientation = self.get_mouth_orientation()
        self.chain_to_layer = None

        self.init_child_chains()

    ####################################################
    # Child chains

    def get_mouth_orientation(self):
        jaw_axis = self.get_bone(self.base_bone).y_axis.copy()
        jaw_axis[2] = 0

        return matrix_from_axis_pair(jaw_axis, (0,0,1), 'z').to_quaternion()

    def init_child_chains(self):
        self.child_chains = [
            rig
            for rig in self.rigify_children
            if isinstance(rig, BasicChainRig) and get_name_side_z(rig.base_bone) != SideZ.MIDDLE
        ]

        self.corners = { Side.LEFT: [], Side.RIGHT: [], SideZ.TOP: [], SideZ.BOTTOM: [] }

    def is_corner_node(self, node):
        siblings = [ n for n in node.get_merged_siblings() if n.rig in self.child_chains ]

        sides_x = set(n.name_split[1] for n in siblings)
        sides_z = set(n.name_split[2] for n in siblings)

        if {SideZ.BOTTOM, SideZ.TOP}.issubset(sides_z):
            if Side.LEFT in sides_x:
                return Side.LEFT
            else:
                return Side.RIGHT

        if {Side.LEFT, Side.RIGHT}.issubset(sides_x):
            if SideZ.TOP in sides_z:
                return SideZ.TOP
            else:
                return SideZ.BOTTOM

        return None

    def arrange_child_chains(self):
        if self.chain_to_layer is not None:
            return

        self.num_layers = len(self.corners[SideZ.TOP])

        for k, v in self.corners.items():
            if len(v) == 0:
                self.raise_error("Could not find all mouth corners")
            if len(v) != self.num_layers:
                self.raise_error(
                    "Mouth corner counts differ: {} vs {}",
                    [n.name for n in v], [n.name for n in self.corners[SideZ.TOP]]
                )

        # Find inner top/bottom corners
        anchor = self.corners[SideZ.BOTTOM][0].point
        inner_top = min(self.corners[SideZ.TOP], key=lambda p: (p.point - anchor).length)

        anchor = inner_top.point
        inner_bottom = min(self.corners[SideZ.BOTTOM], key=lambda p: (p.point - anchor).length)

        # Compute the mouth space
        self.mouth_center = center = (inner_top.point + inner_bottom.point) / 2

        matrix = self.mouth_orientation.to_matrix().to_4x4()
        matrix.translation = center
        self.to_mouth_space = matrix.inverted()

        # Build a mapping of child chain to layer
        self.chain_to_layer = {}
        self.chain_sides = {}

        for k,v in list(self.corners.items()):
            self.corners[k] = ordered = sorted(v, key=lambda p: (p.point - center).length)

            side_set = set()

            for i, node in enumerate(ordered):
                for sibling in node.get_merged_siblings():
                    if sibling.rig in self.child_chains:
                        cur_layer = self.chain_to_layer.get(sibling.rig)

                        if cur_layer is not None and cur_layer != i:
                            self.raise_error("Conflicting mouth chain layer on {}: {} and {}", sibling.rig.base_bone, i, cur_layer)

                        self.chain_to_layer[sibling.rig] = i
                        side_set.add(sibling.rig)

            self.chain_sides[k] = side_set

        for child in self.child_chains:
            if child not in self.chain_to_layer:
                self.raise_error("Could not determine chain layer on {}", child.base_bone)

        if not self.chain_sides[Side.LEFT].isdisjoint(self.chain_sides[Side.RIGHT]):
            self.raise_error("Left/right conflict in mouth")
        if not self.chain_sides[SideZ.TOP].isdisjoint(self.chain_sides[SideZ.BOTTOM]):
            self.raise_error("Top/bottom conflict in mouth")

        # Find left/right direction
        pt = self.to_mouth_space @ self.corners[Side.LEFT][0].point

        self.left_sign = 1 if pt.x > 0 else -1

        for node in self.corners[Side.LEFT]:
            if (self.to_mouth_space @ node.point).x * self.left_sign <= 0:
                self.raise_error("Bad left corner {}", node.name)

        for node in self.corners[Side.RIGHT]:
            if (self.to_mouth_space @ node.point).x * self.left_sign >= 0:
                self.raise_error("Bad right corner {}", node.name)

    def get_node_parent_bones(self, node):
        self.arrange_child_chains()

        layer = self.chain_to_layer[node.rig]

        if node.rig in self.chain_sides[SideZ.TOP]:
            side_mch = self.bones.mch.top[layer]
        else:
            side_mch = self.bones.mch.bottom[layer]

        pt_x = (self.to_mouth_space @ node.point).x
        side = Side.LEFT if pt_x * self.left_sign >= 0 else Side.RIGHT

        corner_x = (self.to_mouth_space @ self.corners[side][layer].point).x
        factor = math.sqrt(1 - clamp(pt_x / corner_x) ** 2)

        return [ (side_mch, factor), (self.bones.mch.middle[layer], 1-factor) ]

    def get_corner_parent_bone(self, node, side_z):
        self.arrange_child_chains()

        layer = self.chain_to_layer[node.rig]

        if side_z == SideZ.TOP:
            return self.bones.mch.top[layer]
        elif side_z == SideZ.BOTTOM:
            return self.bones.mch.bottom[layer]
        else:
            return self.bones.mch.middle[layer]


    ####################################################
    # Control nodes

    def get_parent_for_name(self, name, parent_bone):
        if parent_bone == self.base_bone:
            side = get_name_side_z(name)
            if side == SideZ.TOP:
                return LazyRef(self.bones.mch, 'top', 0)
            if side == SideZ.BOTTOM:
                return LazyRef(self.bones.mch, 'bottom', 0)

        return parent_bone

    def get_child_chain_parent(self, rig, parent_bone):
        return self.get_parent_for_name(rig.base_bone, parent_bone)

    def build_control_node_parent(self, node, parent_bone):
        if node.rig in self.child_chains:
            corner = self.is_corner_node(node)
            if corner:
                if node.merged_master not in self.corners[corner]:
                    self.corners[corner].append(node.merged_master)

                side = SideZ.MIDDLE if corner in {Side.LEFT, Side.RIGHT} else corner

                return ControlBoneParentOrg(LazyRef(self.get_corner_parent_bone, node.merged_master, side))

            else:
                return ControlBoneParentArmature(
                    self, node,
                    bones=LazyRef(self.get_node_parent_bones, node),
                    orientation=self.mouth_orientation,
                )

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

        self.get_bone(self.bones.ctrl.master).lock_scale = (True, True, True)

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

        self.arrange_child_chains()

        mch.lock = self.copy_bone(org, make_derived_name(org, 'mch', '_lock'), scale=1/2, parent=True)

        mch.top = map_list(self.make_mch_top_bone, range(self.num_layers), repeat(org))
        mch.bottom = map_list(self.make_mch_bottom_bone, range(self.num_layers), repeat(org))
        mch.middle = map_list(self.make_mch_middle_bone, range(self.num_layers), repeat(org))

    def make_mch_top_bone(self, i, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '_top'), scale=1/4, parent=True)

    def make_mch_bottom_bone(self, i, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '_bottom'), scale=1/3, parent=True)

    def make_mch_middle_bone(self, i, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '_middle'), scale=2/3, parent=True)

    @stage.parent_bones
    def parent_mch_lock_bones(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl

        for mid, top in zip(mch.middle, mch.top):
            self.set_bone_parent(mid, top)

        for bottom in mch.bottom[1:]:
            self.set_bone_parent(bottom, ctrl.master)

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

        # Lock position
        self.make_constraint(
            mch.lock, 'COPY_TRANSFORMS', ctrl.master,
            influence=self.params.jaw_locked_influence,
        )

        # Innermost top bone
        con = self.make_constraint(mch.top[0], 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])

        # Innermost bottom bone
        self.make_constraint(
            mch.bottom[0], 'COPY_TRANSFORMS', ctrl.master,
            influence=self.params.jaw_mouth_influence,
        )

        con = self.make_constraint(mch.bottom[0], 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])

        # Outer layer bones
        coeff = self.params.jaw_secondary_influence

        for i, name in enumerate(mch.top[1:]):
            self.make_constraint(name, 'COPY_TRANSFORMS', mch.top[0], influence=coeff ** (1+i))

        for i, name in enumerate(mch.bottom[1:]):
            self.make_constraint(name, 'COPY_TRANSFORMS', mch.bottom[0], influence=coeff ** (1+i))

        for mid, bottom in zip(mch.middle, mch.bottom):
            self.make_constraint(mid, 'COPY_TRANSFORMS', bottom, influence=0.5)


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

        params.jaw_secondary_influence = bpy.props.FloatProperty(
            name = "Secondary Influence Falloff",
            default = 0.5, min = 0, max = 1,
            description = "Reduction factor for each level of secondary chains"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "jaw_mouth_influence", slider=True)
        layout.prop(params, "jaw_locked_influence", slider=True)
        layout.prop(params, "jaw_secondary_influence", slider=True)


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
