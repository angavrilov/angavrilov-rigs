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
import enum

from mathutils import Vector, Quaternion
from itertools import count
from collections import defaultdict
from string import Template

from rigify.utils.rig import get_rigify_type
from rigify.utils.errors import MetarigError
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name, get_name_base_and_sides, change_name_side, Side, SideZ
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis
from rigify.utils.widgets_basic import create_cube_widget
from rigify.utils.mechanism import MechanismUtilityMixin
from rigify.utils.misc import force_lazy

from rigify.base_rig import BaseRig, LazyRigComponent, stage

from .node_merger import MainMergeNode, QueryMergeNode


class ControlNodeLayer(enum.IntEnum):
    FREE         = 0
    MIDDLE_PIVOT = 100
    TWEAK        = 200


def _get_parent_rigs(rig):
    result = []
    while rig:
        result.append(rig)
        rig = rig.rigify_parent
    return result


class ControlBoneNode(MainMergeNode, MechanismUtilityMixin):
    """Node representing controls of skin chain rigs."""

    merge_domain = 'ControlNetNode'

    def __init__(
        self, rig, org, name, *, point=None,
        size=None, can_merge=True,
        needs_parent=False, needs_reparent=False,
        layer=ControlNodeLayer.FREE, index=None,
        ):
        assert isinstance(rig, BaseSkinChainRig)

        super().__init__(rig, name, point or rig.get_bone(org).head)

        self.org = org
        self.can_merge = can_merge

        self.name_split = get_name_base_and_sides(name)

        self.name_merged = None
        self.name_merged_split = None

        self.size = size or rig.get_bone(org).length
        self.layer = layer
        self.rotation = None

        # Parent mechanism generator for this node
        self.node_parent = None
        # Create the parent mechanism even if not master
        self.node_needs_parent = needs_parent
        # If this node's own parent mechanism differs from master, generate a conversion bone
        self.node_needs_reparent = needs_reparent

        # Generate the control as MCH unless merged
        self.hide_lone_control = False

        # For use by the owner rig: index in chain
        self.index = index
        # If this node is the end of a chain, points to the next one
        self.chain_end_neighbor = None

    def can_merge_into(self, other):
        # Only merge up the layers (towards more mechanism)
        return (
            self.can_merge and
            (self.layer <= other.layer or not other.can_merge) and
            super().can_merge_into(other)
        )

    def get_merge_priority(self, other):
        # Prefer closest layer
        return 1000 - abs(self.layer - other.layer)

    def is_better_cluster(self, other):
        # Prefer bones that have strictly more parents
        my_parents = list(reversed(_get_parent_rigs(self.rig.rigify_parent)))
        other_parents = list(reversed(_get_parent_rigs(other.rig.rigify_parent)))

        if len(my_parents) > len(other_parents) and my_parents[0:len(other_parents)] == other_parents:
            return True
        if len(other_parents) > len(my_parents) and other_parents[0:len(other_parents)] == my_parents:
            return False

        # Prefer middle chains
        side_x_my, side_z_my = map(abs, self.name_split[1:])
        side_x_other, side_z_other = map(abs, other.name_split[1:])

        if ((side_x_my < side_x_other and side_z_my <= side_z_other) or
            (side_x_my <= side_x_other and side_z_my < side_z_other)):
            return True
        if ((side_x_my > side_x_other and side_z_my >= side_z_other) or
            (side_x_my >= side_x_other and side_z_my > side_z_other)):
            return False

        return False

    def merge_done(self):
        if self.is_master_node:
            self.parent_subrig_cache = []

        super().merge_done()

        self.find_mirror_siblings()

    def find_mirror_siblings(self):
        self.mirror_siblings = {}
        self.mirror_sides_x = set()
        self.mirror_sides_z = set()

        for node in self.get_merged_siblings():
            if node.name_split[0] == self.name_split[0]:
                self.mirror_siblings[node.name_split] = node
                self.mirror_sides_x.add(node.name_split[1])
                self.mirror_sides_z.add(node.name_split[2])

        assert self.mirror_siblings[self.name_split] is self

        # Remove sides that merged with a mirror from the name
        side_x = Side.MIDDLE if len(self.mirror_sides_x) > 1 else self.name_split[1]
        side_z = SideZ.MIDDLE if len(self.mirror_sides_z) > 1 else self.name_split[2]

        self.name_merged = change_name_side(self.name, side=side_x, side_z=side_z)
        self.name_merged_split = (self.name_split[0], side_x, side_z)

    def get_best_mirror(self):
        base, side, sidez = self.name_split

        for flip in [(base, -side, -sidez), (base, -side, sidez), (base, side, -sidez)]:
            mirror = self.mirror_siblings.get(flip, None)
            if mirror and mirror is not self:
                return mirror

        return None

    @classmethod
    def build_parent_for_node(cls, node, cache=None):
        assert node.rig.generator.stage == 'initialize'

        # Build the parent
        result = node.rig.build_control_node_parent(node)

        for rig in reversed(node.rig.get_all_parent_skin_rigs()):
            result = rig.extend_control_node_parent(result, node)

        # Remove duplicates
        cache = cache or node.merged_master.parent_subrig_cache

        for previous in cache:
            if previous == result:
                result = previous
                break
        else:
            result.enable_component()
            cache.append(result)

        return result

    def build_parent(self):
        if not self.node_parent:
            self.node_parent = self.build_parent_for_node(self)

        return self.node_parent

    def get_rotation(self):
        if self.rotation is None:
            self.rotation = self.rig.get_final_control_node_rotation()

        return self.rotation

    def initialize(self):
        if self.is_master_node:
            sibling_list = self.mirror_siblings.values()

            # Compute orientation
            self.rotation = sum((node.get_rotation() for node in sibling_list), Quaternion((0,0,0,0))).normalized()
            self.size = sum(node.size for node in sibling_list) / len(sibling_list)

            self.matrix = self.rotation.to_matrix().to_4x4()
            self.matrix.translation = self.point

            # Create parents
            self.node_parent_list = [ node.build_parent() for node in sibling_list ]

            if all(parent == self.node_parent for parent in self.node_parent_list):
                self.use_mix_parent = False
                self.node_parent_list = [ self.node_parent ]
            else:
                self.use_mix_parent = True

        # All nodes
        if self.node_needs_parent or self.node_needs_reparent:
            self.build_parent()

    @property
    def control_bone(self):
        return self.merged_master._control_bone

    def make_bone(self, name, scale, *, orientation=None):
        name = self.rig.copy_bone(self.org, name)

        if orientation is not None:
            matrix = orientation.to_matrix().to_4x4()
            matrix.translation = self.merged_master.point
        else:
            matrix = self.merged_master.matrix

        bone = self.rig.get_bone(name)
        bone.matrix = matrix
        bone.length = self.merged_master.size * scale

        return name

    def is_reparent_needed(self):
        master = self.merged_master

        return self.node_needs_reparent and (master.use_mix_parent or self.node_parent != master.node_parent)

    def generate_bones(self):
        if self.is_master_node:
            name = self.name_merged

            if self.hide_lone_control and not self.merged:
                name = make_derived_name(name, 'mch')

            self._control_bone = self.make_bone(name, 1)

            if self.use_mix_parent:
                self.mix_parent_bone = self.make_bone(make_derived_name(self._control_bone, 'mch', '_mix_parent'), 1/2)

        # All nodes
        if self.node_needs_reparent:
            if self.is_reparent_needed():
                self.reparent_bone = self.make_bone(make_derived_name(self.name, 'mch', '_reparent'), 1/3)
            else:
                self.reparent_bone = self.control_bone

    def parent_bones(self):
        if self.is_master_node:
            if self.use_mix_parent:
                self.rig.set_bone_parent(self._control_bone, self.mix_parent_bone, inherit_scale='AVERAGE')
                self.rig.generator.disable_auto_parent(self.mix_parent_bone)
            else:
                self.rig.set_bone_parent(self._control_bone, self.node_parent_list[0].output_bone, inherit_scale='AVERAGE')

        # All nodes
        if self.is_reparent_needed():
            self.rig.set_bone_parent(self.reparent_bone, self.node_parent.output_bone, inherit_scale='AVERAGE')

    def rig_bones(self):
        if self.is_master_node:
            if self.use_mix_parent:
                targets = [ parent.output_bone for parent in self.node_parent_list ]
                self.make_constraint(self.mix_parent_bone, 'ARMATURE', targets=targets, use_deform_preserve_volume=True)

            for rig in reversed(self.rig.get_all_parent_skin_rigs()):
                rig.extend_control_node_rig(self)

        # All nodes
        if self.is_reparent_needed():
            self.make_constraint(self.reparent_bone, 'COPY_TRANSFORMS', self.control_bone)

    def generate_widgets(self):
        if self.is_master_node:
            best = min(self.get_merged_siblings(), key=lambda n: n.layer)
            best.rig.make_control_node_widget(best)


class ControlBoneParentOrg:
    """Control node parent generator wrapping a single ORG bone."""

    def __init__(self, org):
        self.output_bone = org

    def enable_component(self):
        pass

    def __eq__(self, other):
        return isinstance(other, ControlBoneParentOrg) and self.output_bone == other.output_bone


class LazyRef:
    """Hashable lazy reference to a bone. When called, evaluates (foo, 'a', 'b'...) as foo('a','b') or foo.a.b."""

    def __init__(self, first, *args):
        self.first = first
        self.args = tuple(args)
        self.first_hashable = first.__hash__ is not None

    def __repr__(self):
        return 'LazyRef{}'.format(tuple(self.first, *self.args))

    def __eq__(self, other):
        return (
            isinstance(other, LazyRef) and
            (self.first == other.first if self.first_hashable else self.first is other.first) and
            self.args == other.args
        )

    def __hash__(self):
        return (hash(self.first) if self.first_hashable else hash(id(self.first))) ^ hash(self.args)

    def __call__(self):
        first = self.first
        if callable(first):
            return first(*self.args)

        for item in self.args:
            first = getattr(first, item)
        return first


class ControlBoneParentOffset(LazyRigComponent):
    """
    Parent mechanism generator that offsets the control's location.

    Supports Copy Transforms (Local) constraints and location drivers.
    Multiple offsets can be accumulated in the same generator, which
    will automatically create as many bones as needed.
    """

    @classmethod
    def wrap(cls, owner, parent, *constructor_args):
        if isinstance(parent, ControlBoneParentOffset):
            return parent

        return cls(owner, parent, *constructor_args)

    def __init__(self, rig, parent, node):
        super().__init__(rig)
        self.parent = parent
        self.node = node
        self.copy_local = {}
        self.add_local = {}
        self.add_orientations = {}

    def add_copy_local_location(self, target, *, influence=1, influence_expr=None, influence_vars={}):
        if target not in self.copy_local:
            self.copy_local[target] = [0, [], []]

        if influence_expr:
            self.copy_local[target][1].append((influence_expr, influence_vars))
        elif callable(influence):
            self.copy_local[target][2].append(influence)
        else:
            self.copy_local[target][0] += influence

    def add_location_driver(self, orientation, index, expression, variables):
        assert isinstance(variables, dict)

        key = tuple(round(x*10000) for x in orientation)

        if key not in self.add_local:
            self.add_orientations[key] = orientation
            self.add_local[key] = ([], [], [])

        self.add_local[key][index].append((expression, variables))

    def __eq__(self, other):
        return (
            isinstance(other, ControlBoneParentOffset) and
            self.parent == other.parent and
            self.copy_local == other.copy_local and
            self.add_local == other.add_local
        )

    @property
    def output_bone(self):
        return self.mch_bones[-1] if self.mch_bones else self.parent.output_bone

    def generate_bones(self):
        self.mch_bones = []

        if self.copy_local or self.add_local:
            mch_name = make_derived_name(self.node.name, 'mch', '_poffset')

            if self.add_local:
                for key in self.add_local:
                    self.mch_bones.append(self.node.make_bone(mch_name, 1/4, orientation=self.add_orientations[key]))
            else:
                self.mch_bones.append(self.node.make_bone(mch_name, 1/4))

    def parent_bones(self):
        if self.mch_bones:
            self.owner.set_bone_parent(self.mch_bones[0], self.parent.output_bone)
            self.owner.parent_bone_chain(self.mch_bones, use_connect=False)

    def compile_driver(self, items):
        variables = {}
        expressions = []

        for expr, varset in items:
            template = Template(expr)
            varmap = {}

            try:
                template.substitute({k:'' for k in varset})
            except Exception as e:
                self.owner.raise_error('Invalid driver expression: {}\nError: {}', expr, e)

            # Merge variables
            for name, desc in varset.items():
                # Check if the variable is used.
                try:
                    template.substitute({k:'' for k in varset if k != name})
                    continue
                except KeyError:
                    pass

                # descriptors may not be hashable, so linear search
                for vn, vdesc in variables.items():
                    if vdesc == desc:
                        varmap[name] = vn
                        break
                else:
                    new_name = name
                    if new_name in variables:
                        for i in count(1):
                            new_name = '%s_%d' % (name, i)
                            if new_name not in variables:
                                break
                    variables[new_name] = desc
                    varmap[name] = new_name

            expressions.append(template.substitute(varmap))

        if len(expressions) > 1:
            final_expr = '+'.join('('+expr+')' for expr in expressions)
        else:
            final_expr = expressions[0]

        return final_expr, variables

    def rig_bones(self):
        if self.copy_local:
            mch = self.mch_bones[0]
            for target, (influence, drivers, lazyinf) in self.copy_local.items():
                influence += sum(map(force_lazy, lazyinf))

                con = self.make_constraint(
                    mch, 'COPY_LOCATION', target, use_offset=True,
                    target_space='OWNER_LOCAL', owner_space='LOCAL', influence=influence,
                )

                if drivers:
                    if influence > 0:
                        drivers.append((str(influence), {}))

                    expr, variables = self.compile_driver(drivers)
                    self.make_driver(con, 'influence', expression=expr, variables=variables)

        if self.add_local:
            for mch, (key, specs) in zip(self.mch_bones, self.add_local.items()):
                for index, vals in enumerate(specs):
                    if vals:
                        expr, variables = self.compile_driver(vals)
                        self.make_driver(mch, 'location', index=index, expression=expr, variables=variables)


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

    def extend_control_node_rig(self, node):
        pass

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
    """Skin chain rig with an option to choose which parent's orientation to use for controls."""

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

