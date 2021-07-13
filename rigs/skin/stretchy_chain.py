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

from itertools import count, repeat
from mathutils import Vector, Matrix
from bl_math import clamp

from rigify.utils.rig import connected_children_names
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, align_bone_roll
from rigify.utils.misc import map_list, LazyRef
from rigify.utils.mechanism import driver_var_transform

from rigify.base_rig import stage

from .skin_nodes import ControlBoneNode, ControlNodeLayer, ControlNodeIcon
from .skin_parents import ControlBoneWeakParentLayer, ControlBoneParentOffset
from .basic_chain import Rig as BasicChainRig


ControlLayersOption.SKIN_PRIMARY = ControlLayersOption(
    'skin_primary', toggle_default=False,
    toggle_name="Primary Control Layers",
    description="Layers for the primary controls to be on",
)

ControlLayersOption.SKIN_SECONDARY = ControlLayersOption(
    'skin_secondary', toggle_default=False,
    toggle_name="Secondary Control Layers",
    description="Layers for the secondary controls to be on",
)


class Rig(BasicChainRig):
    """
    Skin chain that propagates motion of its end and middle controls, resulting in
    stretching the whole chain rather than just immediately connected chain segments.
    """

    min_chain_length = 2

    def initialize(self):
        if len(self.bones.org) < self.min_chain_length:
            self.raise_error(
                "Input to rig type must be a chain of {} or more bones.", self.min_chain_length)

        super().initialize()

        orgs = self.bones.org

        self.pivot_pos = self.params.skin_chain_pivot_pos

        if not (0 <= self.pivot_pos < len(orgs)):
            self.raise_error('Invalid middle control position: {}', self.pivot_pos)

        bone_lengths = [self.get_bone(org).length for org in orgs]

        self.chain_lengths = [sum(bone_lengths[0:i]) for i in range(len(orgs)+1)]

        if not self.params.skin_chain_falloff_length:
            self.pivot_base = self.get_bone(orgs[0]).head
            self.pivot_vector = self.get_bone(orgs[-1]).tail - self.pivot_base
            self.pivot_length = self.pivot_vector.length
            self.pivot_vector.normalize()

        if self.pivot_pos:
            pivot_point = self.get_bone(orgs[self.pivot_pos]).head
            self.middle_pivot_factor = self.get_pivot_projection(pivot_point, self.pivot_pos)

    def get_pivot_projection(self, pos, index):
        if self.params.skin_chain_falloff_length:
            return self.chain_lengths[index] / self.chain_lengths[-1]
        else:
            return clamp((pos - self.pivot_base).dot(self.pivot_vector) / self.pivot_length)

    ####################################################
    # CONTROL NODES

    def make_control_node(self, i, org, is_end):
        node = super().make_control_node(i, org, is_end)

        if i == 0 or i == self.num_orgs:
            node.layer = ControlNodeLayer.FREE
            node.icon = ControlNodeIcon.FREE
            if i == 0:
                node.node_needs_reparent = self.use_falloff_curve(0)
            else:
                node.node_needs_reparent = self.use_falloff_curve(2)
        elif i == self.pivot_pos:
            node.layer = ControlNodeLayer.MIDDLE_PIVOT
            node.icon = ControlNodeIcon.MIDDLE_PIVOT
            node.node_needs_reparent = self.use_falloff_curve(1)
        else:
            node.layer = ControlNodeLayer.TWEAK
            node.icon = ControlNodeIcon.TWEAK

        return node

    def use_falloff_curve(self, idx):
        return self.params.skin_chain_falloff[idx] > -10

    def apply_falloff_curve(self, factor, idx):
        weight = self.params.skin_chain_falloff[idx]

        if self.params.skin_chain_falloff_spherical[idx]:
            # circular falloff
            if weight >= 0:
                p = 2 ** weight
                return (1 - (1 - factor) ** p) ** (1/p)
            else:
                p = 2 ** -weight
                return 1 - (1 - factor ** p) ** (1/p)
        else:
            # parabolic falloff
            return 1 - (1 - factor) ** (2 ** weight)

    def apply_falloff_start(self, factor):
        return self.apply_falloff_curve(factor, 0)

    def apply_falloff_middle(self, factor):
        return self.apply_falloff_curve(factor, 1)

    def apply_falloff_end(self, factor):
        return self.apply_falloff_curve(factor, 2)

    def extend_control_node_parent(self, parent, node):
        if node.rig != self or node.index in (0, self.num_orgs):
            return parent

        parent = ControlBoneParentOffset(self, node, parent)
        factor = self.get_pivot_projection(node.point, node.index)

        if self.use_falloff_curve(0):
            parent.add_copy_local_location(
                LazyRef(self.control_nodes[0], 'reparent_bone'),
                influence=self.apply_falloff_start(1 - factor),
            )

        if self.use_falloff_curve(2):
            parent.add_copy_local_location(
                LazyRef(self.control_nodes[-1], 'reparent_bone'),
                influence=self.apply_falloff_end(factor),
            )

        if self.pivot_pos and node.index != self.pivot_pos and self.use_falloff_curve(1):
            if node.index < self.pivot_pos:
                factor = factor / self.middle_pivot_factor
            else:
                factor = (1 - factor) / (1 - self.middle_pivot_factor)

            parent.add_copy_local_location(
                LazyRef(self.control_nodes[self.pivot_pos], 'reparent_bone'),
                influence=self.apply_falloff_middle(clamp(factor)),
            )

        if node.index != self.pivot_pos and self.params.skin_chain_falloff_to_controls:
            if self.params.skin_chain_falloff_twist or self.params.skin_chain_falloff_scale:
                parent = ControlBoneChainPropagate(self, node, parent)

        return parent

    def get_control_node_layers(self, node):
        layers = None

        if self.pivot_pos and node.index == self.pivot_pos:
            layers = ControlLayersOption.SKIN_SECONDARY.get(self.params)

        if not layers and node.index in (0, self.num_orgs, self.pivot_pos):
            layers = ControlLayersOption.SKIN_PRIMARY.get(self.params)

        return layers or super().get_control_node_layers(node)

    ####################################################
    # B-Bone handle MCH

    def rig_mch_handle_user(self, i, mch, prev_node, node, next_node, pre):
        super().rig_mch_handle_user(i, mch, prev_node, node, next_node, pre)

        self.rig_propagate(mch, node)

    def rig_propagate(self, mch, node):
        # Interpolate chain twist between pivots
        if node.index not in (0, self.num_orgs, self.pivot_pos):
            index1, index2, factor = self.get_propagate_spec(node)

            if self.params.skin_chain_falloff_twist:
                self.rig_propagate_twist(mch, index1, index2, factor)

            if self.use_scale and self.params.skin_chain_falloff_scale:
                self.rig_propagate_scale(mch, index1, index2, factor)

    def get_propagate_spec(self, node):
        index1 = 0
        index2 = self.num_orgs

        len_cur = self.chain_lengths[node.index]
        len_end = self.chain_lengths[-1]

        if self.pivot_pos:
            len_pivot = self.chain_lengths[self.pivot_pos]

            if node.index < self.pivot_pos:
                factor = len_cur / len_pivot
                index2 = self.pivot_pos
            else:
                factor = (len_cur - len_pivot) / (len_end - len_pivot)
                index1 = self.pivot_pos
        else:
            factor = len_cur / len_end

        return index1, index2, factor

    def rig_propagate_twist(self, mch, index1, index2, factor):
        handles = self.get_all_mch_handles()
        handles_pre = self.get_all_mch_handles_pre()

        variables = {
            'y1': driver_var_transform(
                self.obj, handles[index1], type='ROT_Y',
                space='LOCAL', rotation_mode='SWING_TWIST_Y'
            ),
            'y2': driver_var_transform(
                self.obj, handles[index2], type='ROT_Y',
                space='LOCAL', rotation_mode='SWING_TWIST_Y'
            ),
        }

        if handles_pre[index1] != handles[index1]:
            variables['p1'] = driver_var_transform(
                self.obj, handles_pre[index1], type='ROT_Y',
                space='LOCAL', rotation_mode='SWING_TWIST_Y'
            )
            expr1 = 'y1-p1'
        else:
            expr1 = 'y1'

        if handles_pre[index2] != handles[index2]:
            variables['p2'] = driver_var_transform(
                self.obj, handles_pre[index2], type='ROT_Y',
                space='LOCAL', rotation_mode='SWING_TWIST_Y'
            )
            expr2 = 'y2-p2'
        else:
            expr2 = 'y2'

        bone = self.get_bone(mch)
        bone.rotation_mode = 'YXZ'

        self.make_driver(
            bone, 'rotation_euler', index=1,
            expression='lerp({},{},{})'.format(expr1, expr2, clamp(factor)),
            variables=variables
        )

    def rig_propagate_scale(self, mch, index1, index2, factor, use_y=False):
        handles = self.get_all_mch_handles()

        self.make_constraint(
            mch, 'COPY_SCALE', handles[index1], space='LOCAL',
            use_x=True, use_y=use_y, use_z=True,
            use_offset=True, power=clamp(1-factor)
        )
        self.make_constraint(
            mch, 'COPY_SCALE', handles[index2], space='LOCAL',
            use_x=True, use_y=use_y, use_z=True,
            use_offset=True, power=clamp(factor)
        )

    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.skin_chain_pivot_pos = bpy.props.IntProperty(
            name='Middle Control Position',
            default=0,
            min=0,
            description='Position of the middle control, disabled if zero'
        )

        params.skin_chain_falloff_spherical = bpy.props.BoolVectorProperty(
            size=3,
            name='Spherical Falloff',
            default=(False, False, False),
            description='Falloff curve tries to form a circle at +1 instead of a parabola',
        )

        params.skin_chain_falloff = bpy.props.FloatVectorProperty(
            size=3,
            name='Control Falloff',
            default=(0.0, 1.0, 0.0),
            soft_min=-2, min=-10, soft_max=2,
            description='Falloff curve coefficient: 0 is linear, and higher value is wider influence. Set to -10 to disable influence completely',
        )

        params.skin_chain_falloff_length = bpy.props.BoolProperty(
            name='Falloff Along Chain Curve',
            default=False,
            description='Falloff is computed along the curve of the chain, instead of projecting on the axis connecting the start and end points',
        )

        params.skin_chain_falloff_twist = bpy.props.BoolProperty(
            name='Propagate Twist',
            default=True,
            description='Propagate twist from main controls throughout the chain',
        )

        params.skin_chain_falloff_scale = bpy.props.BoolProperty(
            name='Propagate Scale',
            default=False,
            description='Propagate scale from main controls throughout the chain',
        )

        params.skin_chain_falloff_to_controls = bpy.props.BoolProperty(
            name='Propagate To Controls',
            default=False,
            description='Expose scale and/or twist propagated to tweak controls to be seen as ' +
                        'parent motion by glue or other chains using Merge Parent Rotation And ' +
                        'Scale. Otherwise it is only propagated internally within this chain',
        )

        ControlLayersOption.SKIN_PRIMARY.add_parameters(params)
        ControlLayersOption.SKIN_SECONDARY.add_parameters(params)

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "skin_chain_pivot_pos")

        col = layout.column(align=True)

        row = col.row(align=True)
        row.label(text="Falloff:")

        for i in range(3):
            row2 = row.row(align=True)
            row2.active = i != 1 or params.skin_chain_pivot_pos > 0
            row2.prop(params, "skin_chain_falloff", text="", index=i)
            row2.prop(params, "skin_chain_falloff_spherical", text="", icon='SPHERECURVE', index=i)

        col.prop(params, "skin_chain_falloff_length")

        row = col.split(factor=0.25)
        row.label(text="Propagate:")
        row = row.row(align=True)
        row.prop(params, "skin_chain_falloff_twist", text="Twist", toggle=True)
        row.prop(params, "skin_chain_falloff_scale", text="Scale", toggle=True)
        row.prop(params, "skin_chain_falloff_to_controls", text="To Controls", toggle=True)

        ControlLayersOption.SKIN_PRIMARY.parameters_ui(layout, params)

        if params.skin_chain_pivot_pos > 0:
            ControlLayersOption.SKIN_SECONDARY.parameters_ui(layout, params)

        super().parameters_ui(layout, params)


class ControlBoneChainPropagate(ControlBoneWeakParentLayer):
    """
    Parent mechanism generator that propagates chain twist/scale.
    """
    inherit_scale_mode = 'FULL'

    def __eq__(self, other):
        return (
            isinstance(other, ControlBoneChainPropagate) and
            self.parent == other.parent and
            self.rig == other.rig and
            self.node.index == other.node.index
        )

    def generate_bones(self):
        handle = self.rig.bones.mch.handles[self.node.index]
        self.output_bone = self.copy_bone(handle, make_derived_name(handle, 'mch', '_parent'))

    def parent_bones(self):
        self.set_bone_parent(self.output_bone, self.parent.output_bone, inherit_scale='AVERAGE')

    def rig_bones(self):
        self.rig.rig_propagate(self.output_bone, self.node)


def create_sample(obj):
    from rigify.rigs.basic.copy_chain import create_sample as inner
    obj.pose.bones[inner(obj)["bone.01"]].rigify_type = 'skin.stretchy_chain'
