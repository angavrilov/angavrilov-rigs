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

from rigify.utils.rig import connected_children_names
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, align_bone_roll
from rigify.utils.widgets_basic import create_cube_widget, create_sphere_widget
from rigify.utils.misc import map_list

from rigify.base_rig import stage

from .skin_rigs import BaseSkinChainRigWithRotationOption, ControlBoneNode


class Rig(BaseSkinChainRigWithRotationOption):
    """Skin chain with completely independent control nodes."""

    def find_org_bones(self, bone):
        return [bone.name] + connected_children_names(self.obj, bone.name)

    def initialize(self):
        super().initialize()

        self.chain_priority = self.params.skin_chain_priority

        self.bbone_segments = self.params.bbones
        self.use_bbones = self.bbone_segments > 1
        self.use_connect_mirror = self.params.skin_chain_connect_mirror
        self.use_connect_ends = self.params.skin_chain_connect_ends

        orgs = self.bones.org

        self.num_orgs = len(orgs)
        self.length = sum([self.get_bone(b).length for b in orgs]) / len(orgs)
        self.chain_rot = compute_chain_orientation(self.obj, orgs).to_quaternion()

    def get_control_node_rotation(self):
        return self.chain_rot

    ####################################################
    # CONTROL NODES

    @stage.initialize
    def init_control_nodes(self):
        orgs = self.bones.org

        self.control_nodes = nodes = [
            # Bone head nodes
            *map_list(self.make_control_node, count(0), orgs, repeat(False)),
            # Tail of the final bone
            self.make_control_node(len(orgs), orgs[-1], True),
        ]

        nodes[0].chain_end_neighbor = nodes[1]
        nodes[-1].chain_end_neighbor = nodes[-2]

    def make_control_node(self, i, org, is_end):
        bone = self.get_bone(org)
        name = make_derived_name(org, 'ctrl', '_end' if is_end else '')
        pos = bone.tail if is_end else bone.head
        return ControlBoneNode(self, org, name, point=pos, size=self.length/3, index=i)

    def make_control_node_widget(self, node):
        create_sphere_widget(self.obj, node.control_bone)

    ####################################################
    # BONES
    #
    # mch:
    #   handles[]
    #     B-Bone handles.
    #   handles_pre[] (optional, may be copy of handles[])
    #     Separate mechanism bones that emulates Auto handle behavior.
    # deform[]:
    #   Deformation B-Bones.
    #
    ####################################################

    ####################################################
    # B-Bone handle MCH

    # Generate two layers of handle bones - used by other rigs
    use_pre_handles = False

    def get_connected_node(self, node):
        if self.use_connect_mirror:
            mirror = node.get_best_mirror()
            if mirror and mirror.chain_end_neighbor and isinstance(mirror.rig, Rig):
                if mirror.rig.use_connect_mirror:
                    return mirror, mirror.chain_end_neighbor

        if self.use_connect_ends:
            starts = []
            ends = []

            for sibling in node.get_merged_siblings():
                if isinstance(sibling.rig, Rig) and sibling.chain_end_neighbor and sibling.rig.use_connect_ends:
                    if sibling.index == 0:
                        starts.append(sibling)
                    else:
                        ends.append(sibling)

            if len(starts) == 1 and len(ends) == 1:
                assert node == starts[0] or node == ends[0]
                link = starts[0] if node == ends[0] else ends[0]
                return link, link.chain_end_neighbor

        return None, None

    def get_node_chain_with_mirror(self):
        nodes = self.control_nodes
        prev_link, prev_node = self.get_connected_node(nodes[0])
        next_link, next_node = self.get_connected_node(nodes[-1])

        # Optimize connect next by sharing last handle mch
        self.next_chain_rig = None

        if next_link and next_link.index == 0:
            self.next_chain_rig = next_link.rig
            return [ prev_node, *nodes ]

        return [ prev_node, *nodes, next_node ]

    def get_all_mch_handles(self):
        if self.next_chain_rig:
            return self.bones.mch.handles + [ self.next_chain_rig.bones.mch.handles[0] ]
        else:
            return self.bones.mch.handles

    def get_all_mch_handles_pre(self):
        if self.next_chain_rig:
            return self.bones.mch.handles_pre + [ self.next_chain_rig.bones.mch.handles_pre[0] ]
        else:
            return self.bones.mch.handles_pre

    @stage.generate_bones
    def make_mch_handle_bones(self):
        if self.use_bbones:
            mch = self.bones.mch
            chain = self.get_node_chain_with_mirror()

            mch.handles = map_list(self.make_mch_handle_bone, count(0), chain, chain[1:], chain[2:])

            if self.use_pre_handles:
                mch.handles_pre = map_list(self.make_mch_pre_handle_bone, count(0), mch.handles)
            else:
                mch.handles_pre = mch.handles

    def make_mch_handle_bone(self, i, prev_node, node, next_node):
        name = self.copy_bone(node.org, make_derived_name(node.name, 'mch', '_handle'))

        hstart = prev_node or node
        hend = next_node or node
        haxis = (hend.point - hstart.point).normalized()

        bone = self.get_bone(name)
        bone.tail = bone.head + haxis * self.length * 3/4

        align_bone_roll(self.obj, name, node.org)
        return name

    def make_mch_pre_handle_bone(self, i, handle):
        return self.copy_bone(handle, make_derived_name(handle, 'mch', '_pre'))

    @stage.parent_bones
    def parent_mch_handle_bones(self):
        if self.use_bbones:
            mch = self.bones.mch

            if self.use_pre_handles:
                for pre in mch.handles_pre:
                    self.set_bone_parent(pre, self.rig_parent_bone, inherit_scale='AVERAGE')

            for handle in mch.handles:
                self.set_bone_parent(handle, self.rig_parent_bone, inherit_scale='AVERAGE')

    @stage.rig_bones
    def rig_mch_handle_bones(self):
        if self.use_bbones:
            mch = self.bones.mch
            chain = self.get_node_chain_with_mirror()

            for args in zip(count(0), mch.handles_pre, chain, chain[1:], chain[2:]):
                self.rig_mch_handle_auto(*args)

            for args in zip(count(0), mch.handles, chain, chain[1:], chain[2:], mch.handles_pre):
                self.rig_mch_handle_user(*args)

    def rig_mch_handle_auto(self, i, mch, prev_node, node, next_node):
        hstart = prev_node or node
        hend = next_node or node

        # Emulate auto handle
        self.make_constraint(mch, 'COPY_LOCATION', hstart.control_bone, name='locate_prev')
        self.make_constraint(mch, 'DAMPED_TRACK', hend.control_bone, name='track_next')

    def rig_mch_handle_user(self, i, mch, prev_node, node, next_node, pre):
        if pre != mch:
            self.make_constraint(
                mch, 'COPY_TRANSFORMS', pre, name='copy_pre',
                space='LOCAL', mix_mode='BEFORE_FULL',
            )

        # Apply user rotation and scale
        self.make_constraint(
            mch, 'COPY_TRANSFORMS', node.control_bone, name='copy_user',
            target_space='OWNER_LOCAL', owner_space='LOCAL',
            mix_mode='BEFORE_FULL',
        )

        # Remove any shear created by previous step
        self.make_constraint(mch, 'LIMIT_ROTATION', name='remove_shear')


    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        orgs = self.bones.org
        self.set_bone_parent(orgs[0], self.rig_parent_bone, inherit_scale='AVERAGE')
        self.parent_bone_chain(orgs, use_connect=True, inherit_scale='AVERAGE')


    @stage.rig_bones
    def rig_org_chain(self):
        for args in zip(count(0), self.bones.org, self.control_nodes, self.control_nodes[1:]):
            self.rig_org_bone(*args)

    def rig_org_bone(self, i, org, node, next_node):
        if i == 0:
            self.make_constraint(org, 'COPY_LOCATION', node.control_bone)

        self.make_constraint(org, 'STRETCH_TO', next_node.control_bone, keep_axis='SWING_Y')


    ##############################
    # Deform chain

    @stage.generate_bones
    def make_deform_chain(self):
        self.bones.deform = map_list(self.make_deform_bone, count(0), self.bones.org)

    def make_deform_bone(self, i, org):
        name = self.copy_bone(org, make_derived_name(org, 'def'), bbone=True)
        self.get_bone(name).bbone_segments = self.bbone_segments
        return name

    @stage.parent_bones
    def parent_deform_chain(self):
        deform = self.bones.deform

        self.set_bone_parent(deform[0], self.rig_parent_bone, inherit_scale='AVERAGE')
        self.parent_bone_chain(deform, use_connect=True, inherit_scale='AVERAGE')

        if self.use_bbones:
            handles = self.get_all_mch_handles()

            for name, start_handle, end_handle in zip(deform, handles, handles[1:]):
                bone = self.get_bone(name)
                bone.bbone_handle_type_start = 'TANGENT'
                bone.bbone_custom_handle_start = self.get_bone(start_handle)
                bone.bbone_handle_type_end = 'TANGENT'
                bone.bbone_custom_handle_end = self.get_bone(end_handle)

    @stage.rig_bones
    def rig_deform_chain(self):
        for args in zip(count(0), self.bones.deform, self.bones.org):
            self.rig_deform_bone(*args)

    def rig_deform_bone(self, i, deform, org):
        self.make_constraint(deform, 'COPY_TRANSFORMS', org)


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.bbones = bpy.props.IntProperty(
            name        = 'B-Bone Segments',
            default     = 10,
            min         = 1,
            description = 'Number of B-Bone segments'
        )

        params.skin_chain_connect_mirror = bpy.props.BoolProperty(
            name        = 'Connect With Mirror',
            default     = True,
            description = 'Create a smooth B-Bone transition if an end of the chain meets its mirror'
        )

        params.skin_chain_connect_ends = bpy.props.BoolProperty(
            name        = 'Connect Matching Ends',
            default     = False,
            description = 'Create a smooth B-Bone transition if an end of the chain meets another chain going in the same direction'
        )

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "bbones")

        col = layout.column()
        col.active = params.bbones > 1
        col.prop(params, "skin_chain_connect_mirror")
        col.prop(params, "skin_chain_connect_ends")

        super().parameters_ui(layout, params)

        layout.prop(params, "skin_chain_priority")


def compute_chain_orientation(obj, bone_names):
    """
    Compute the orientation matrix with x axis perpendicular
    to the primary plane in which the bones lie.
    """
    pb = obj.pose.bones
    first_bone = pb[bone_names[0]]
    last_bone = pb[bone_names[-1]]

    y_axis = last_bone.tail - first_bone.head

    if y_axis.length < 1e-4:
        y_axis = (last_bone.head - first_bone.tail).normalized()
    else:
        y_axis.normalize()

    x_axis = first_bone.y_axis.normalized().cross(y_axis)

    if x_axis.length < 1e-4:
        z_axis = first_bone.x_axis.cross(y_axis).normalized()

        return Matrix((y_axis.cross(z_axis), y_axis, z_axis)).transposed()
    else:
        x_axis.normalize()

        return Matrix((x_axis, y_axis, x_axis.cross(y_axis))).transposed()
