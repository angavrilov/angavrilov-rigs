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

from mathutils import Vector, Quaternion
from itertools import count

from rigify.utils.rig import get_rigify_type
from rigify.utils.errors import MetarigError
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name, get_name_base_and_sides
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis
from rigify.utils.widgets_basic import create_cube_widget
from rigify.utils.mechanism import MechanismUtilityMixin

from rigify.base_rig import BaseRig, LazyRigComponent, stage

from .node_merger import MainMergeNode, QueryMergeNode


class ControlBoneNode(MainMergeNode, MechanismUtilityMixin):
    merge_domain = 'ControlNetNode'

    def __init__(self, rig, org, name, *, point=None, size=None, can_merge=True, needs_parent=False):
        assert isinstance(rig, BaseSkinChainRig)

        super().__init__(rig, name, point or rig.get_bone(org).head)

        self.org = org
        self.can_merge = can_merge

        self.name_split = get_name_base_and_sides(name)
        self.parent_skin_rigs = rig.get_all_parent_skin_rigs()

        self.size = size or rig.get_bone(org).length
        self.rotation = None
        self.node_parent = None
        self.node_needs_parent = False # always generate the parent

        self.chain_neighbor = None

    def can_merge_into(self, other):
        return self.can_merge and super().can_merge_into(other)

    def find_mirror_siblings(self):
        self.mirror_siblings = {}

        for node in self.get_merged_siblings():
            if node.name_split[0] == self.name_split[0]:
                self.mirror_siblings[node.name_split] = node

        assert self.mirror_siblings[self.name_split] is self

    def get_best_mirror(self):
        base, side, sidez = self.name_split

        for flip in [(base, -side, -sidez), (base, -side, sidez), (base, side, -sidez)]:
            mirror = self.mirror_siblings.get(flip, None)
            if mirror and mirror is not self:
                return mirror

        return None

    def merge_done(self):
        if not self.merged_into:
            self.parent_subrig_cache = []

        super().merge_done()

        self.find_mirror_siblings()

    def build_parent(self):
        if self.node_parent:
            return self.node_parent

        assert self.rig.generator.stage == 'initialize'

        # Build the parent
        result = self.rig.build_control_node_parent(self)

        for rig in reversed(self.parent_skin_rigs):
            result = rig.extend_control_node_parent(result, self)

        # Remove duplicates
        cache = self.merged_master.parent_subrig_cache

        for previous in cache:
            if previous == result:
                result = previous
                break
        else:
            result.enable_component()
            cache.append(result)

        self.node_parent = result
        return result

    def get_rotation(self):
        if self.rotation is None:
            self.rotation = self.rig.get_final_control_node_rotation()

        return self.rotation

    def initialize(self):
        sibling_list = self.mirror_siblings.values()

        # Compute orientation
        self.rotation = sum((node.get_rotation() for node in sibling_list), Quaternion((0,0,0,0))).normalized()
        self.size = sum(node.size for node in sibling_list) / len(sibling_list)

        self.matrix = self.rotation.to_matrix().to_4x4()
        self.matrix.translation = self.point

        # Create parents
        self.node_parent_list = [ node.build_parent() for node in sibling_list ]

        if all(parent == self.node_parent_list[0] for parent in self.node_parent_list):
            self.node_parent_list = self.node_parent_list[0:1]

        for child in self.merged:
            if child.node_needs_parent:
                child.build_parent()

    @property
    def control_bone(self):
        return self.merged_master._control_bone

    def make_bone(self, name, scale):
        name = self.rig.copy_bone(self.org, name)

        bone = self.rig.get_bone(name)
        bone.matrix = self.matrix
        bone.length = self.size * scale

        return name

    def generate_bones(self):
        self._control_bone = self.make_bone(self.name, 1)

        if len(self.node_parent_list) > 1:
            self.mix_parent_bone = self.make_bone(make_derived_name(self.name, 'mch', '_mix_parent'), 1/2)

    def parent_bones(self):
        if len(self.node_parent_list) > 1:
            self.rig.set_bone_parent(self._control_bone, self.mix_parent_bone, inherit_scale='AVERAGE')
            self.rig.generator.disable_auto_parent(self.mix_parent_bone)
        else:
            self.rig.set_bone_parent(self._control_bone, self.node_parent_list[0].output_bone, inherit_scale='AVERAGE')

    def rig_bones(self):
        if len(self.node_parent_list) > 1:
            targets = [ parent.output_bone for parent in self.node_parent_list ]
            self.make_constraint(self.mix_parent_bone, 'ARMATURE', targets=targets, use_deform_preserve_volume=True)

    def generate_widgets(self):
        self.rig.make_control_node_widget(self)


class ControlBoneParentOrg:
    def __init__(self, org):
        self.output_bone = org

    def enable_component(self):
        pass

    def __eq__(self, other):
        return isinstance(other, ControlBoneParentOrg) and self.output_bone == other.output_bone


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

    def build_control_node_parent(self, node):
        return ControlBoneParentOrg(self.base_bone)

    def extend_control_node_parent(self, parent, node):
        return parent

    def get_control_node_rotation(self):
        return self.get_bone(self.base_bone).bone.matrix_local.to_quaternion()


class BaseSkinChainRig(BaseSkinRig):
    """Base type for all rigs that can have control nodes"""

    def build_control_node_parent(self, node):
        if isinstance(self.rigify_parent, BaseSkinRig):
            return self.rigify_parent.build_control_node_parent(node)
        else:
            return ControlBoneParentOrg(self.rig_parent_bone)

    def get_control_node_rotation(self):
        return self.get_bone(self.rig_parent_bone).bone.matrix_local.to_quaternion()

    def get_final_control_node_rotation(self):
        return self.get_control_node_rotation()

    def make_control_node_widget(self, node):
        raise NotImplementedError()


class BaseSkinChainRigWithRotationOption(BaseSkinChainRig):
    def get_final_control_node_rotation(self):
        # Hack: read the raw value without accessing the RNA wrapper
        index = self.params.get("skin_control_rotation_index", 0)
        rig = self

        while index > 0 and rig.rigify_parent:
            rig = rig.rigify_parent
            index -= 1

        if isinstance(rig, BaseSkinRig):
            result = rig.get_control_node_rotation()
        else:
            result = rig.get_bone(rig.base_bone).bone.matrix_local.to_quaternion()

        return result

    __enum_items = []

    @staticmethod
    def parent_enum_items(scene, context):
        pbone = context.active_pose_bone
        if not pbone:
            return

        items = BaseSkinChainRigWithRotationOption.__enum_items
        items.clear()

        while pbone:
            rtype = get_rigify_type(pbone)
            if rtype:
                items.append((pbone.name, '%s (%s)' % (pbone.name, rtype), ''))
            pbone = pbone.parent

        if not items:
            items.append(('unknown', 'unknown', ''))
        return items

    @classmethod
    def add_parameters(self, params):
        params.skin_control_rotation_index = bpy.props.EnumProperty(
            name        = "Control Orientation",
            description = "Select which parent rig provides orientation for the control bones",
            items       = BaseSkinChainRigWithRotationOption.parent_enum_items,
        )

    @classmethod
    def parameters_ui(self, layout, params):
        r = layout.row()
        r.prop(params, "skin_control_rotation_index")

