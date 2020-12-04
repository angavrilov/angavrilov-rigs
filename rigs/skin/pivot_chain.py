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

from itertools import count, repeat
from mathutils import Vector, Matrix
from bl_math import clamp

from rigify.utils.rig import connected_children_names
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, align_bone_roll
from rigify.utils.widgets_basic import create_cube_widget, create_sphere_widget
from rigify.utils.misc import map_list
from rigify.utils.mechanism import driver_var_transform

from rigify.base_rig import stage

from .skin_rigs import ControlBoneNode, ControlNodeLayer, ControlBoneParentOffset, LazyRef
from .basic_chain import Rig as BasicChainRig


class Rig(BasicChainRig):
    """Skin chain with extra pivots."""

    min_chain_length = 2

    def initialize(self):
        if len(self.bones.org) < self.min_chain_length:
            self.raise_error("Input to rig type must be a chain of {} or more bones.", self.min_chain_length)

        super().initialize()

        orgs = self.bones.org

        self.pivot_pos = self.params.skin_chain_pivot_pos

        if not (0 <= self.pivot_pos < len(orgs)):
            self.raise_error('Invalid middle pivot position: {}', self.pivot_pos)

        self.pivot_base = self.get_bone(orgs[0]).head
        self.pivot_vector = self.get_bone(orgs[-1]).tail - self.pivot_base
        self.pivot_length = self.pivot_vector.length
        self.pivot_vector.normalize()

        if self.pivot_pos:
            self.middle_pivot_factor = self.get_pivot_projection(self.get_bone(orgs[self.pivot_pos]).head)

        bone_lengths = [ self.get_bone(org).length for org in orgs ]

        self.chain_lengths = [ sum(bone_lengths[0:i]) for i in range(len(orgs)+1) ]

    def get_pivot_projection(self, pos):
        return (pos - self.pivot_base).dot(self.pivot_vector) / self.pivot_length

    ####################################################
    # CONTROL NODES

    def make_control_node(self, i, org, is_end):
        node = super().make_control_node(i, org, is_end)

        if i == 0 or i == self.num_orgs:
            node.layer = ControlNodeLayer.FREE
            node.size *= 1.5
            node.node_needs_reparent = True
        elif i == self.pivot_pos:
            node.layer = ControlNodeLayer.MIDDLE_PIVOT
            node.node_needs_reparent = True
        else:
            node.layer = ControlNodeLayer.TWEAK

        return node

    def make_control_node_widget(self, node):
        if node.index in (0, self.num_orgs, self.pivot_pos):
            create_cube_widget(self.obj, node.control_bone)
        else:
            create_sphere_widget(self.obj, node.control_bone)

    def apply_falloff_start(self, factor):
        return 1 - (1 - factor) ** self.params.skin_chain_falloff_start

    def apply_falloff_middle(self, factor):
        return 1 - (1 - factor) ** self.params.skin_chain_falloff_middle

    def apply_falloff_end(self, factor):
        return 1 - (1 - factor) ** self.params.skin_chain_falloff_end

    def extend_control_node_parent(self, parent, node):
        if node.index in (0, self.num_orgs):
            return parent

        parent = ControlBoneParentOffset.wrap(self, parent, node)
        factor = clamp(self.get_pivot_projection(node.point))

        parent.add_copy_local_location(
            LazyRef(self.control_nodes[0], 'reparent_bone'),
            influence = self.apply_falloff_start(1 - factor),
        )
        parent.add_copy_local_location(
            LazyRef(self.control_nodes[-1], 'reparent_bone'),
            influence = self.apply_falloff_end(factor),
        )

        if self.pivot_pos and node.index != self.pivot_pos:
            if node.index < self.pivot_pos:
                factor = factor / self.middle_pivot_factor
            else:
                factor = (1 - factor) / (1 - self.middle_pivot_factor)

            parent.add_copy_local_location(
                LazyRef(self.control_nodes[self.pivot_pos], 'reparent_bone'),
                influence = self.apply_falloff_middle(clamp(factor)),
            )

        return parent


    ####################################################
    # B-Bone handle MCH

    def rig_mch_handle_user(self, i, mch, prev_node, node, next_node):
        super().rig_mch_handle_user(i, mch, prev_node, node, next_node)

        # Interpolate chain twist between pivots
        if node.index not in (0, self.num_orgs, self.pivot_pos):
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

            variables = {
                'y1': driver_var_transform(
                    self.obj, self.bones.mch.handles[index1], type='ROT_Y',
                    space='LOCAL', rotation_mode='SWING_TWIST_Y'
                ),
                'y2': driver_var_transform(
                    self.obj, self.bones.mch.handles[index2], type='ROT_Y',
                    space='LOCAL', rotation_mode='SWING_TWIST_Y'
                ),
            }

            bone = self.get_bone(mch)
            bone.rotation_mode = 'YXZ'

            self.make_driver(
                bone, 'rotation_euler', index=1,
                expression='lerp(y1,y2,{})'.format(clamp(factor)),
                variables=variables
            )


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.skin_chain_pivot_pos = bpy.props.IntProperty(
            name='Middle Pivot Position',
            default=0,
            min=0,
            description='Position of the middle pivot, disabled if zero'
        )

        params.skin_chain_falloff_start = bpy.props.FloatProperty(
            name='Start Control Falloff',
            default=1,
            min=0,
            description='Falloff curve power of the chain start control; higher value is wider influence',
        )

        params.skin_chain_falloff_middle = bpy.props.FloatProperty(
            name='Middle Control Falloff',
            default=2,
            min=0,
            description='Falloff curve power of the chain middle pivot control; higher value is wider influence',
        )

        params.skin_chain_falloff_end = bpy.props.FloatProperty(
            name='End Control Falloff',
            default=1,
            min=0,
            description='Falloff curve power of the chain end control; higher value is wider influence',
        )

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "skin_chain_pivot_pos")

        row = layout.row(align=True)
        row.label(text="Falloff:")
        row.prop(params, "skin_chain_falloff_start", text="")

        row2 = row.row(align=True)
        row2.active = params.skin_chain_pivot_pos > 0
        row2.prop(params, "skin_chain_falloff_middle", text="")

        row.prop(params, "skin_chain_falloff_end", text="")

        super().parameters_ui(layout, params)
