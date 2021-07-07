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

from rigify.utils.naming import make_derived_name
from rigify.utils.misc import force_lazy, LazyRef

from rigify.base_rig import BaseRig, stage

from .skin_parents import ControlBoneParentOrg


class BaseSkinRig(BaseRig):
    """Base type for all rigs involved in the skin system"""
    def initialize(self):
        self.rig_parent_bone = self.get_bone_parent(self.base_bone)

    def get_parent_skin_rig(self):
        parent = self.rigify_parent

        while parent:
            if isinstance(parent, BaseSkinRig):
                return parent
            parent = parent.rigify_parent

        return None

    def get_all_parent_skin_rigs(self):
        items = []
        current = self
        while current:
            items.append(current)
            current = current.get_parent_skin_rig()
        return items

    def get_child_chain_parent_next(self, rig):
        "Delegate parenting of the child chain to the parent rig."
        if isinstance(self.rigify_parent, BaseSkinRig):
            return self.rigify_parent.get_child_chain_parent(rig, self.rig_parent_bone)
        else:
            return self.rig_parent_bone

    def get_child_chain_parent(self, rig, parent_bone):
        # May return LazyRef if necessary
        return parent_bone

    def build_control_node_parent_next(self, node):
        "Delegate parenting of the control node to the parent rig."
        if isinstance(self.rigify_parent, BaseSkinRig):
            return self.rigify_parent.build_control_node_parent(node, self.rig_parent_bone)
        else:
            return ControlBoneParentOrg(self.rig_parent_bone)

    def build_control_node_parent(self, node, parent_bone):
        "Called when a child rig delegates control node parenting."
        return ControlBoneParentOrg(self.get_child_chain_parent(node.rig, parent_bone))

    def extend_control_node_parent(self, parent, node):
        return parent

    def extend_control_node_parent_post(self, parent, node):
        return parent

    def extend_control_node_rig(self, node):
        pass


def get_bone_quaternion(obj, bone):
    return obj.pose.bones[bone].bone.matrix_local.to_quaternion()


class BaseSkinChainRig(BaseSkinRig):
    """Base type for all rigs that can have control nodes"""

    chain_priority = 0

    def parent_bones(self):
        self.rig_parent_bone = force_lazy(self.get_child_chain_parent_next(self))

    def build_own_control_node_parent(self, node):
        "Called to build the primary parent of nodes owned by this rig."
        return self.build_control_node_parent_next(node)

    def get_final_control_node_rotation(self, node):
        return self.get_control_node_rotation(node)

    def get_control_node_rotation(self, node):
        return get_bone_quaternion(self.obj, self.base_bone)

    def get_control_node_layers(self, node):
        return self.get_bone(self.base_bone).bone.layers

    def make_control_node_widget(self, node):
        raise NotImplementedError()

    @classmethod
    def add_parameters(self, params):
        params.skin_chain_priority = bpy.props.IntProperty(
            name='Chain Priority',
            min=-10, max=10, default=0,
            description='When merging controls, chains with higher priority always win'
        )


class BaseSkinChainRigWithRotationOption(BaseSkinChainRig):
    """Skin chain rig with an option to choose which parent's orientation to use for controls."""

    def get_final_control_node_rotation(self, node):
        bone_name = self.params.skin_control_orientation_bone

        if bone_name:
            try:
                org_name = make_derived_name(bone_name, 'org')

                if org_name not in self.obj.pose.bones:
                    org_name = bone_name

                return get_bone_quaternion(self.obj, org_name)

            except KeyError:
                self.raise_error('Could not find orientation bone {}', bone_name)

        else:
            return self.get_control_node_rotation(node)

    @classmethod
    def add_parameters(self, params):
        params.skin_control_orientation_bone = bpy.props.StringProperty(
            name        = "Orientation Bone",
            description = "If set, control orientation is taken from the specified bone",
        )

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        from rigify.operators.copy_mirror_parameters import make_copy_parameter_button

        row = layout.row()
        row.prop_search(params, "skin_control_orientation_bone", bpy.context.active_object.pose, "bones", text="Orientation")

        make_copy_parameter_button(
            row, "skin_control_orientation_bone", mirror_bone=True,
            base_class=BaseSkinChainRigWithRotationOption
        )
