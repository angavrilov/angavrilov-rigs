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
from rigify.utils.layers import set_bone_layers
from rigify.utils.naming import make_derived_name, get_name_base_and_sides, change_name_side, Side, SideZ, mirror_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, BoneUtilityMixin, set_bone_widget_transform
from rigify.utils.widgets_basic import create_cube_widget, create_sphere_widget
from rigify.utils.mechanism import MechanismUtilityMixin
from rigify.utils.misc import force_lazy

from rigify.base_rig import BaseRig, LazyRigComponent, stage

from .node_merger import MainMergeNode, QueryMergeNode


class ControlNodeLayer(enum.IntEnum):
    FREE         = 0
    MIDDLE_PIVOT = 10
    TWEAK        = 20

class ControlNodeIcon(enum.IntEnum):
    TWEAK        = 0
    MIDDLE_PIVOT = 1
    FREE         = 2
    CUSTOM       = 3

class ControlNodeEnd(enum.IntEnum):
    START        = -1
    MIDDLE       = 0
    END          = 1

def _get_parent_rigs(rig):
    result = []
    while rig:
        result.append(rig)
        rig = rig.rigify_parent
    return result


class ControlBoneNode(MainMergeNode, MechanismUtilityMixin, BoneUtilityMixin):
    """Node representing controls of skin chain rigs."""

    merge_domain = 'ControlNetNode'

    def __init__(
        self, rig, org, name, *, point=None, size=None,
        needs_parent=False, needs_reparent=False, allow_scale=False,
        chain_end=ControlNodeEnd.MIDDLE,
        layer=ControlNodeLayer.FREE, index=None, icon=ControlNodeIcon.TWEAK,
        ):
        assert isinstance(rig, BaseSkinChainRig)

        super().__init__(rig, name, point or rig.get_bone(org).head)

        self.org = org

        self.name_split = get_name_base_and_sides(name)

        self.name_merged = None
        self.name_merged_split = None

        self.size = size or rig.get_bone(org).length
        self.layer = layer
        self.icon = icon
        self.rotation = None
        self.chain_end = chain_end

        # Parent mechanism generator for this node
        self.node_parent = None
        # Create the parent mechanism even if not master
        self.node_needs_parent = needs_parent
        # If this node's own parent mechanism differs from master, generate a conversion bone
        self.node_needs_reparent = needs_reparent

        # Generate the control as a MCH bone to hide it from the user
        self.hide_control = False
        # Unlock scale channels
        self.allow_scale = allow_scale

        # For use by the owner rig: index in chain
        self.index = index
        # If this node is the end of a chain, points to the next one
        self.chain_end_neighbor = None

    def can_merge_into(self, other):
        # Only merge up the layers (towards more mechanism)
        dprio = self.rig.chain_priority - other.rig.chain_priority
        return (
            dprio <= 0 and
            (self.layer <= other.layer or dprio < 0) and
            super().can_merge_into(other)
        )

    def get_merge_priority(self, other):
        # Prefer higher and closest layer
        if self.layer <= other.layer:
            return -abs(self.layer - other.layer)
        else:
            return -abs(self.layer - other.layer) - 100

    def is_better_cluster(self, other):
        # Prefer bones that have strictly more parents
        my_parents = list(reversed(_get_parent_rigs(self.rig.rigify_parent)))
        other_parents = list(reversed(_get_parent_rigs(other.rig.rigify_parent)))

        if len(my_parents) > len(other_parents) and my_parents[0:len(other_parents)] == other_parents:
            return True
        if len(other_parents) > len(my_parents) and other_parents[0:len(other_parents)] == my_parents:
            return False

        # Prefer side chains
        side_x_my, side_z_my = map(abs, self.name_split[1:])
        side_x_other, side_z_other = map(abs, other.name_split[1:])

        if ((side_x_my < side_x_other and side_z_my <= side_z_other) or
            (side_x_my <= side_x_other and side_z_my < side_z_other)):
            return False
        if ((side_x_my > side_x_other and side_z_my >= side_z_other) or
            (side_x_my >= side_x_other and side_z_my > side_z_other)):
            return True

        return False

    def merge_done(self):
        if self.is_master_node:
            self.parent_subrig_cache = []
            self.parent_subrig_names = {}
            self.reparent_requests = []
            self.used_parents = {}

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

    def build_parent_for_node(self, node, use_parent=False):
        assert self.rig.generator.stage == 'initialize'

        # Build the parent
        result = node.rig.build_own_control_node_parent(node)
        parents = node.rig.get_all_parent_skin_rigs()

        for rig in reversed(parents):
            result = rig.extend_control_node_parent(result, node)

        for rig in parents:
            result = rig.extend_control_node_parent_post(result, node)

        result = self.intern_parent(node, result)
        result.is_parent_frozen = True

        if use_parent:
            self.register_use_parent(result)

        return result

    def intern_parent(self, node, parent):
        if id(parent) in self.parent_subrig_names:
            return parent

        cache = self.parent_subrig_cache

        for previous in cache:
            if previous == parent:
                previous.is_parent_frozen = True
                return previous

        cache.append(parent)
        self.parent_subrig_names[id(parent)] = node.name

        if isinstance(parent, ControlBoneParentLayer):
            parent.parent = self.intern_parent(node, parent.parent)

        return parent

    def build_parent(self):
        if not self.node_parent:
            self.node_parent = self.merged_master.build_parent_for_node(self)

        return self.node_parent

    def register_use_parent(self, parent):
        parent.is_parent_frozen = True
        self.merged_master.used_parents[id(parent)] = parent

    def request_reparent(self, parent):
        master = self.merged_master
        requests = master.reparent_requests

        if parent not in requests:
            if parent != master.node_parent or master.use_mix_parent:
                master.register_use_parent(master.node_parent)

            master.register_use_parent(parent)
            requests.append(parent)

    def get_rotation(self):
        if self.rotation is None:
            self.rotation = self.rig.get_final_control_node_rotation(self)

        return self.rotation

    def initialize(self):
        if self.is_master_node:
            sibling_list = self.get_merged_siblings()
            mirror_sibling_list = self.mirror_siblings.values()

            # Compute size
            best = max(sibling_list, key=lambda n: n.icon)
            best_mirror = best.mirror_siblings.values()

            self.size = sum(node.size for node in best_mirror) / len(best_mirror)

            # Compute orientation
            self.rotation = sum(
                    (node.get_rotation() for node in mirror_sibling_list),
                    Quaternion((0,0,0,0))
                ).normalized()

            self.matrix = self.rotation.to_matrix().to_4x4()
            self.matrix.translation = self.point

            # Create parents
            self.node_parent_list = [ node.build_parent() for node in mirror_sibling_list ]

            if all(parent == self.node_parent for parent in self.node_parent_list):
                self.use_mix_parent = False
                self.node_parent_list = [ self.node_parent ]
            else:
                self.use_mix_parent = True

            self.has_weak_parent = isinstance(self.node_parent, ControlBoneWeakParentLayer)
            self.node_parent_base = ControlBoneWeakParentLayer.strip(self.node_parent)

            self.node_parent_list = [ ControlBoneWeakParentLayer.strip(p) for p in self.node_parent_list ]

            for parent in self.node_parent_list:
                self.register_use_parent(parent)

        # All nodes
        if self.node_needs_parent or self.node_needs_reparent:
            parent = self.build_parent()
            if self.node_needs_reparent:
                self.request_reparent(parent)

    def prepare_bones(self):
        # Activate parent components once all reparents are registered
        if self.is_master_node:
            for parent in self.used_parents.values():
                parent.enable_component()

            self.used_parents = None

    @property
    def control_bone(self):
        return self.merged_master._control_bone

    def get_reparent_bone(self, parent):
        return self.reparent_bones[id(parent)]

    @property
    def reparent_bone(self):
        return self.merged_master.get_reparent_bone(self.node_parent)

    def make_bone(self, name, scale, *, rig=None, orientation=None):
        name = (rig or self).copy_bone(self.org, name)

        if orientation is not None:
            matrix = orientation.to_matrix().to_4x4()
            matrix.translation = self.merged_master.point
        else:
            matrix = self.merged_master.matrix

        bone = self.get_bone(name)
        bone.matrix = matrix
        bone.length = self.merged_master.size * scale

        return name

    def find_master_name_node(self):
        # Chain end nodes have sub-par names, so try to find another chain
        if self.chain_end == ControlNodeEnd.END:
            siblings = [
                node for node in self.get_merged_siblings()
                if self.mirror_sides_x.issubset(node.mirror_sides_x)
                and self.mirror_sides_z.issubset(node.mirror_sides_z)
            ]

            candidates = [ node for node in siblings if node.chain_end == ControlNodeEnd.START ]

            if not candidates:
                candidates = [ node for node in siblings if node.chain_end == ControlNodeEnd.MIDDLE ]

            if candidates:
                return min(candidates, key=lambda c: (-c.rig.chain_priority, c.name_merged))

        return self

    def generate_bones(self):
        if self.is_master_node:
            # Make control bone
            self._control_bone = self.make_master_bone()

            # Make mix parent if needed
            self.reparent_bones = {}

            if self.use_mix_parent:
                self.mix_parent_bone = self.make_bone(make_derived_name(self._control_bone, 'mch', '_mix_parent'), 1/2)
            else:
                self.reparent_bones[id(self.node_parent)] = self._control_bone

            self.use_weak_parent = False

            # Make requested reparents
            for parent in self.reparent_requests:
                if id(parent) not in self.reparent_bones:
                    parent_name = self.parent_subrig_names[id(parent)]
                    self.reparent_bones[id(parent)] = self.make_bone(make_derived_name(parent_name, 'mch', '_reparent'), 1/3)
                    self.use_weak_parent = self.has_weak_parent

            if self.use_weak_parent:
                self.weak_parent_bone = self.make_bone(make_derived_name(self._control_bone, 'mch', '_weak_parent'), 1/2)

    def make_master_bone(self):
        choice = self.find_master_name_node()
        name = choice.name_merged

        if self.hide_control:
            name = make_derived_name(name, 'mch')

        return choice.make_bone(name, 1)

    def parent_bones(self):
        if self.is_master_node:
            if self.use_mix_parent:
                self.set_bone_parent(self._control_bone, self.mix_parent_bone, inherit_scale='AVERAGE')
                self.rig.generator.disable_auto_parent(self.mix_parent_bone)
            else:
                self.set_bone_parent(self._control_bone, self.node_parent_list[0].output_bone, inherit_scale='AVERAGE')

            if self.use_weak_parent:
                self.set_bone_parent(
                    self.weak_parent_bone, self.node_parent.output_bone,
                    inherit_scale=self.node_parent.inherit_scale_mode
                )

            for parent in self.reparent_requests:
                bone = self.reparent_bones[id(parent)]
                if bone != self._control_bone:
                    self.set_bone_parent(bone, parent.output_bone, inherit_scale='AVERAGE')

    def configure_bones(self):
        if self.is_master_node:
            if not any(node.allow_scale for node in self.get_merged_siblings()):
                self.get_bone(self.control_bone).lock_scale = (True, True, True)

        layers = self.rig.get_control_node_layers(self)
        if layers:
            bone = self.get_bone(self.control_bone).bone
            set_bone_layers(bone, layers, not self.is_master_node)

    def rig_bones(self):
        if self.is_master_node:
            if self.use_mix_parent:
                targets = [ parent.output_bone for parent in self.node_parent_list ]
                self.make_constraint(self.mix_parent_bone, 'ARMATURE', targets=targets, use_deform_preserve_volume=True)

            for rig in reversed(self.rig.get_all_parent_skin_rigs()):
                rig.extend_control_node_rig(self)

            reparent_source = self.control_bone

            if self.use_weak_parent:
                reparent_source = self.weak_parent_bone

                self.make_constraint(reparent_source, 'COPY_TRANSFORMS', self.control_bone, space='LOCAL')

                set_bone_widget_transform(self.obj, self.control_bone, reparent_source)

            for parent in self.reparent_requests:
                bone = self.reparent_bones[id(parent)]
                if bone != self._control_bone:
                    self.make_constraint(bone, 'COPY_TRANSFORMS', reparent_source)

    def generate_widgets(self):
        if self.is_master_node:
            best = max(self.get_merged_siblings(), key=lambda n: n.icon)

            if best.icon == ControlNodeIcon.TWEAK:
                create_sphere_widget(self.obj, self.control_bone)
            elif best.icon in (ControlNodeIcon.MIDDLE_PIVOT, ControlNodeIcon.FREE):
                create_cube_widget(self.obj, self.control_bone)
            else:
                best.rig.make_control_node_widget(best)


class ControlQueryNode(QueryMergeNode, MechanismUtilityMixin, BoneUtilityMixin):
    """Node representing controls of skin chain rigs."""

    merge_domain = 'ControlNetNode'

    def __init__(self, rig, org, *, name=None, point=None, find_highest_layer=False):
        assert isinstance(rig, BaseSkinRig)

        super().__init__(rig, name or org, point or rig.get_bone(org).head)

        self.org = org
        self.find_highest_layer = find_highest_layer

    def can_merge_into(self, other):
        return True

    def get_merge_priority(self, other):
        return other.layer if self.find_highest_layer else -other.layer

    @property
    def merged_master(self):
        return self.matched_nodes[0]

    @property
    def control_bone(self):
        return self.merged_master.control_bone


class ControlBoneParentBase(LazyRigComponent):
    rigify_sub_object_run_late = True

    # This parent cannot be merged with other wrappers?
    is_parent_frozen = False

    def __init__(self, rig, node):
        super().__init__(node)
        self.rig = rig
        self.node = node

    def __eq__(self, other):
        raise NotImplementedError()


class ControlBoneParentOrg:
    """Control node parent generator wrapping a single ORG bone."""

    is_parent_frozen = True

    def __init__(self, org):
        self._output_bone = org

    @property
    def output_bone(self):
        return force_lazy(self._output_bone)

    def enable_component(self):
        pass

    def __eq__(self, other):
        return isinstance(other, ControlBoneParentOrg) and self._output_bone == other._output_bone


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
            if isinstance(first, (dict, list)):
                first = first[item]
            else:
                first = getattr(first, item)

        return first


class ControlBoneParentArmature(ControlBoneParentBase):
    """Control node parent generator using Armature to parent the bone."""

    def __init__(self, rig, node, *, bones, orientation=None, copy_scale=None, copy_rotation=None):
        super().__init__(rig, node)
        self.bones = bones
        self.orientation = orientation
        self.copy_scale = copy_scale
        self.copy_rotation = copy_rotation
        if copy_scale or copy_rotation:
            self.is_parent_frozen = True

    def __eq__(self, other):
        return (
            isinstance(other, ControlBoneParentArmature) and
            self.node.point == other.node.point and
            self.orientation == other.orientation and
            self.bones == other.bones and
            self.copy_scale == other.copy_scale and
            self.copy_rotation == other.copy_rotation
        )

    def generate_bones(self):
        self.output_bone = self.node.make_bone(make_derived_name(self.node.name, 'mch', '_arm'), 1/4, rig=self.rig)

        self.rig.generator.disable_auto_parent(self.output_bone)

        if self.orientation:
            matrix = force_lazy(self.orientation).to_matrix().to_4x4()
            matrix.translation = self.node.point
            self.get_bone(self.output_bone).matrix = matrix

    def parent_bones(self):
        self.targets = force_lazy(self.bones)

        assert len(self.targets) > 0

        if len(self.targets) == 1:
            target = force_lazy(self.targets[0])
            if isinstance(target, tuple):
                target = target[0]

            self.set_bone_parent(
                self.output_bone, target,
                inherit_scale='NONE' if self.copy_scale else 'FIX_SHEAR'
            )

    def rig_bones(self):
        if len(self.targets) > 1:
            self.make_constraint(
                self.output_bone, 'ARMATURE', targets=force_lazy(self.bones),
                use_deform_preserve_volume=True
            )

            self.make_constraint(self.output_bone, 'LIMIT_ROTATION')

        if self.copy_rotation:
            self.make_constraint(self.output_bone, 'COPY_ROTATION', self.copy_rotation)
        if self.copy_scale:
            self.make_constraint(self.output_bone, 'COPY_SCALE', self.copy_scale)


class ControlBoneParentLayer(ControlBoneParentBase):
    def __init__(self, rig, node, parent):
        super().__init__(rig, node)
        self.parent = parent

    def enable_component(self):
        self.parent.enable_component()
        super().enable_component()


class ControlBoneWeakParentLayer(ControlBoneParentLayer):
    inherit_scale_mode = 'AVERAGE'

    @staticmethod
    def strip(parent):
        while isinstance(parent, ControlBoneWeakParentLayer):
            parent = parent.parent

        return parent


class ControlBoneParentOffset(ControlBoneParentLayer):
    """
    Parent mechanism generator that offsets the control's location.

    Supports Copy Transforms (Local) constraints and location drivers.
    Multiple offsets can be accumulated in the same generator, which
    will automatically create as many bones as needed.
    """

    @classmethod
    def wrap(cls, owner, parent, node, *constructor_args):
        return cls(owner, node, parent, *constructor_args)

    def __init__(self, rig, node, parent):
        super().__init__(rig, node, parent)
        self.copy_local = {}
        self.add_local = {}
        self.add_orientations = {}
        self.limit_distance = []

    def enable_component(self):
        while isinstance(self.parent, ControlBoneParentOffset) and not self.parent.is_parent_frozen:
            self.prepend_contents(self.parent)
            self.parent = self.parent.parent

        super().enable_component()

    def prepend_contents(self, other):
        for key, val in other.copy_local.items():
            if key not in self.copy_local:
                self.copy_local[key] = val
            else:
                inf, expr, cbs = val
                inf0, expr0, cbs0 = self.copy_local[key]
                self.copy_local[key] = [inf+inf0, expr+expr0, cbs+cbs0]

        for key, val in other.add_orientations.items():
            if key not in self.add_orientations:
                self.add_orientations[key] = val

        for key, val in other.add_local.items():
            if key not in self.add_local:
                self.add_local[key] = val
            else:
                ot0, ot1, ot2 = val
                my0, my1, my2 = self.add_local[key]
                self.add_local[key] = (ot0+my0, ot1+my1, ot2+my2)

        self.limit_distance = other.limit_distance + self.limit_distance

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

    def add_limit_distance(self, target, **kwargs):
        self.limit_distance.append((target, kwargs))

    def __eq__(self, other):
        return (
            isinstance(other, ControlBoneParentOffset) and
            self.parent == other.parent and
            self.copy_local == other.copy_local and
            self.add_local == other.add_local and
            self.limit_distance == other.limit_distance
        )

    @property
    def output_bone(self):
        return self.mch_bones[-1] if self.mch_bones else self.parent.output_bone

    def generate_bones(self):
        self.mch_bones = []
        self.reuse_mch = False

        if self.copy_local or self.add_local or self.limit_distance:
            mch_name = make_derived_name(self.node.name, 'mch', '_poffset')

            if self.add_local:
                for key in self.add_local:
                    self.mch_bones.append(self.node.make_bone(mch_name, 1/4, rig=self.rig, orientation=self.add_orientations[key]))
            else:
                # Try piggybacking on the parent bone if allowed
                if not self.parent.is_parent_frozen:
                    bone = self.get_bone(self.parent.output_bone)
                    if (bone.head - self.node.point).length < 1e-5:
                        self.reuse_mch = True
                        self.mch_bones = [ bone.name ]
                        return

                self.mch_bones.append(self.node.make_bone(mch_name, 1/4, rig=self.rig))

    def parent_bones(self):
        if self.mch_bones:
            if not self.reuse_mch:
                self.rig.set_bone_parent(self.mch_bones[0], self.parent.output_bone)

            self.rig.parent_bone_chain(self.mch_bones, use_connect=False)

    def compile_driver(self, items):
        variables = {}
        expressions = []

        for expr, varset in items:
            template = Template(expr)
            varmap = {}

            try:
                template.substitute({k:'' for k in varset})
            except Exception as e:
                self.rig.raise_error('Invalid driver expression: {}\nError: {}', expr, e)

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
                    target_space='LOCAL', owner_space='LOCAL', influence=influence,
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

        for target, kwargs in self.limit_distance:
            self.make_constraint(self.mch_bones[-1], 'LIMIT_DISTANCE', target, **kwargs)


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
        row = layout.row()
        row.prop_search(params, "skin_control_orientation_bone", bpy.context.active_object.pose, "bones", text="Orientation")

        props = row.operator(POSE_OT_RigifySkinSyncRotationIndex.bl_idname, icon='DUPLICATE', text='')
        props.property_name = "skin_control_orientation_bone"
        props.class_name = __name__ + ':BaseSkinChainRigWithRotationOption'
        props.mirror_bone = True


class POSE_OT_RigifySkinSyncRotationIndex(bpy.types.Operator):
    """Upgrades metarig bones rigify_types"""

    bl_idname = "pose.rigify_skin_sync_rotation_index"
    bl_label = "Copy Option To Selected Rigs"
    bl_description = 'Set all selected metarigs of the same type to this property value'
    bl_options = {'UNDO'}

    property_name: bpy.props.StringProperty(name='Property Name')
    rig_type: bpy.props.StringProperty(name='Rig Name:BaseClass')
    class_name: bpy.props.StringProperty(name='Module:Class', default='')
    mirror_bone: bpy.props.BoolProperty(name='Mirror Bone Name')

    @classmethod
    def poll(cls, context):
        return (
            context.active_object and context.active_object.type == 'ARMATURE'
            and context.active_pose_bone
            and context.active_object.data.get("rig_id") is None
            and get_rigify_type(context.active_pose_bone)
        )

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        import rigify.rig_lists as rig_lists

        if self.class_name:
            import importlib
            module_name, class_name = self.class_name.split(':')

            try:
                filter_rig_class = getattr(importlib.import_module(module_name), class_name)
            except (KeyError, AttributeError, ImportError):
                self.report({'ERROR'}, 'Invalid class spec: ' + self.class_name)
                return {'CANCELLED'}
        else:
            items = self.rig_type.split(':')

            try:
                filter_rig_class = rig_lists.rigs[items[0]]["module"].Rig
            except (KeyError, AttributeError):
                self.report({'ERROR'}, 'Invalid rig type: ' + items[0])
                return {'CANCELLED'}

            if len(items) > 1:
                bases = { c.__name__: c for c in type.mro(filter_rig_class) }
                if items[1] not in bases:
                    self.report({'ERROR'}, 'Invalid rig base class: ' + items[1])
                    return {'CANCELLED'}

                filter_rig_class = bases[items[1]]

        pbone = context.active_pose_bone
        value = getattr(pbone.rigify_parameters, self.property_name)
        name_split = get_name_base_and_sides(pbone.name)

        for sel_pbone in context.selected_pose_bones:
            rig_type = get_rigify_type(sel_pbone)
            if rig_type and sel_pbone != pbone:
                try:
                    rig_class = rig_lists.rigs[rig_type]["module"].Rig
                except (KeyError, AttributeError):
                    continue

                if issubclass(rig_class, filter_rig_class):
                    new_value = value

                    if self.mirror_bone and name_split[1] != Side.MIDDLE and value:
                        sel_split = get_name_base_and_sides(sel_pbone.name)
                        if sel_split[1] == -name_split[1]:
                            new_value = mirror_name(value)

                    setattr(sel_pbone.rigify_parameters, self.property_name, new_value)

        return {'FINISHED'}
