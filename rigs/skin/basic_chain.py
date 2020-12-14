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

from itertools import count, repeat
from mathutils import Vector, Matrix, Quaternion

from math import acos
from bl_math import smoothstep

from rigify.utils.rig import connected_children_names
from rigify.utils.layers import ControlLayersOption
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, align_bone_roll
from rigify.utils.widgets_basic import create_cube_widget, create_sphere_widget
from rigify.utils.misc import map_list

from rigify.base_rig import stage

from .skin_rigs import BaseSkinChainRigWithRotationOption, ControlBoneNode, get_bone_quaternion


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
        self.use_scale = any(self.params.skin_chain_use_scale)
        self.use_reparent_handles = self.params.skin_chain_use_reparent

        orgs = self.bones.org

        self.num_orgs = len(orgs)
        self.length = sum([self.get_bone(b).length for b in orgs]) / len(orgs)

    def get_control_node_rotation(self, node):
        orgs = self.bones.org
        bones = orgs[max(0,node.index-1):node.index+1]
        quats = [ get_bone_quaternion(self.obj, name) for name in bones ]

        return sum(quats, Quaternion((0,0,0,0))).normalized()

    def get_all_controls(self):
        return [ node.control_bone for node in self.control_nodes ]

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
        return ControlBoneNode(
            self, org, name, point=pos, size=self.length/3, index=i,
            allow_scale=self.use_scale, needs_reparent=self.use_reparent_handles,
        )

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
        is_end = 1 if node.index != 0 else 0
        corner = self.params.skin_chain_connect_sharp_angle[is_end]

        if self.use_connect_mirror[is_end]:
            mirror = node.get_best_mirror()
            if mirror and mirror.chain_end_neighbor and isinstance(mirror.rig, Rig):
                s_is_end = 1 if mirror.index != 0 else 0
                if is_end == s_is_end and mirror.rig.use_connect_mirror[is_end]:
                    mirror_corner = mirror.rig.params.skin_chain_connect_sharp_angle[is_end]
                    return mirror, mirror.chain_end_neighbor, (corner + mirror_corner)/2

        if self.use_connect_ends[is_end]:
            groups = ([], [])

            for sibling in node.get_merged_siblings():
                if isinstance(sibling.rig, Rig) and sibling.chain_end_neighbor:
                    s_is_end = 1 if sibling.index != 0 else 0
                    if sibling.rig.use_connect_ends[s_is_end]:
                        groups[s_is_end].append(sibling)

            if len(groups[0]) == 1 and len(groups[1]) == 1:
                assert node == groups[is_end][0]
                link = groups[1 - is_end][0]
                link_corner = link.rig.params.skin_chain_connect_sharp_angle[1 - is_end]
                return link, link.chain_end_neighbor, (corner + link_corner)/2

        return None, None, False

    def get_node_chain_with_mirror(self):
        nodes = self.control_nodes
        prev_link, self.prev_node, self.prev_corner = self.get_connected_node(nodes[0])
        next_link, self.next_node, self.next_corner = self.get_connected_node(nodes[-1])

        # Optimize connect next by sharing last handle mch
        self.next_chain_rig = None

        if next_link and next_link.index == 0:
            self.next_chain_rig = next_link.rig
            return [ self.prev_node, *nodes ]

        return [ self.prev_node, *nodes, self.next_node ]

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
        input_bone = node.reparent_bone if self.use_reparent_handles else node.control_bone

        self.make_constraint(
            mch, 'COPY_TRANSFORMS', input_bone, name='copy_user',
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

                if self.use_scale:
                    bone.bbone_handle_scale_start = self.params.skin_chain_use_scale
                    bone.bbone_handle_scale_end = self.params.skin_chain_use_scale

    @stage.rig_bones
    def rig_deform_chain(self):
        for args in zip(count(0), self.bones.deform, self.bones.org):
            self.rig_deform_bone(*args)

    def rig_deform_bone(self, i, deform, org):
        self.make_constraint(deform, 'COPY_TRANSFORMS', org)

        if i == 0 and self.prev_corner > 1e-3:
            self.make_corner_driver(deform, 'bbone_easein', self.control_nodes[0], self.control_nodes[1], self.prev_node, self.prev_corner)
        elif i == self.num_orgs-1 and self.next_corner > 1e-3:
            self.make_corner_driver(deform, 'bbone_easeout', self.control_nodes[-1], self.control_nodes[-2], self.next_node, self.next_corner)

    def make_corner_driver(self, bbone, field, corner_node, next_node1, next_node2, angle):
        pbone = self.get_bone(bbone)

        a = (corner_node.point - next_node1.point).length
        b = (corner_node.point - next_node2.point).length
        c = (next_node1.point - next_node2.point).length

        varmap = {
            'a': driver_var_distance(self.obj, bone1=corner_node.control_bone, bone2=next_node1.control_bone),
            'b': driver_var_distance(self.obj, bone1=corner_node.control_bone, bone2=next_node2.control_bone),
            'c': driver_var_distance(self.obj, bone1=next_node1.control_bone, bone2=next_node2.control_bone),
        }

        initval = -1+2*smoothstep(-1,1,acos((a*a+b*b-c*c)/max(2*a*b,1e-10))/angle)

        setattr(pbone.bone, field, initval)

        self.make_driver(
            pbone, field,
            expression='%f+2*smoothstep(-1,1,acos((a*a+b*b-c*c)/max(2*a*b,1e-10))/%f)' % (-1-initval, angle),
            variables=varmap
        )

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

        params.skin_chain_use_reparent = bpy.props.BoolProperty(
            name='Merge Parent Rotation And Scale',
            default=False,
            description='When merging controls, apply parent-induced rotation/scale difference',
        )

        params.skin_chain_use_scale = bpy.props.BoolVectorProperty(
            size        = 4,
            name        = 'Use Handle Scale',
            default     = (False, False, False, False),
            description = 'Use control scaling to scale the B-Bone'
        )

        params.skin_chain_connect_mirror = bpy.props.BoolVectorProperty(
            size        = 2,
            name        = 'Connect With Mirror',
            default     = (True, True),
            description = 'Create a smooth B-Bone transition if an end of the chain meets its mirror'
        )

        params.skin_chain_connect_sharp_angle = bpy.props.FloatVectorProperty(
            size        = 2,
            name        = 'Sharpen Corner',
            default     = (0, 0),
            min         = 0,
            max         = math.pi,
            description = 'Create a mechanism to sharpen a connected corner when the angle is below this value',
            unit        = 'ROTATION',
        )

        params.skin_chain_connect_ends = bpy.props.BoolVectorProperty(
            size        = 2,
            name        = 'Connect Matching Ends',
            default     = (False, False),
            description = 'Create a smooth B-Bone transition if an end of the chain meets another chain going in the same direction'
        )

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "bbones")

        col = layout.column()
        col.active = params.bbones > 1

        col.prop(params, "skin_chain_use_reparent")

        row = col.split(factor=0.3)
        row.label(text="Use Scale:")
        row = row.row(align=True)
        row.prop(params, "skin_chain_use_scale", index=0, text="X", toggle=True)
        row.prop(params, "skin_chain_use_scale", index=1, text="Y", toggle=True)
        row.prop(params, "skin_chain_use_scale", index=2, text="Len", toggle=True)
        row.prop(params, "skin_chain_use_scale", index=3, text="Ease", toggle=True)

        row = col.split(factor=0.3)
        row.label(text="Connect Mirror:")
        row = row.row(align=True)
        row.prop(params, "skin_chain_connect_mirror", index=0, text="Start", toggle=True)
        row.prop(params, "skin_chain_connect_mirror", index=1, text="End", toggle=True)

        row = col.split(factor=0.3)
        row.label(text="Connect Next:")
        row = row.row(align=True)
        row.prop(params, "skin_chain_connect_ends", index=0, text="Start", toggle=True)
        row.prop(params, "skin_chain_connect_ends", index=1, text="End", toggle=True)

        row = col.split(factor=0.3)
        row.label(text="Sharpen:")
        row = row.row(align=True)
        row.prop(params, "skin_chain_connect_sharp_angle", index=0, text="Start")
        row.prop(params, "skin_chain_connect_sharp_angle", index=1, text="End")

        super().parameters_ui(layout, params)

        layout.prop(params, "skin_chain_priority")



def driver_var_distance(target, *, bone1=None, target2=None, bone2=None, space1='WORLD', space2='WORLD'):
    """
    Create a Distance driver variable specification.

    Usage:
        make_driver(..., variables=[driver_var_distance(...)])

    Target bone name can be provided via a 'lazy' callable closure without arguments.
    """

    assert space1 in {'WORLD', 'TRANSFORM', 'LOCAL'}
    assert space2 in {'WORLD', 'TRANSFORM', 'LOCAL'}

    target1_map = {
        'id': target,
        'transform_space': space1 + '_SPACE',
    }

    if bone1 is not None:
        target1_map['bone_target'] = bone1

    target2_map = {
        'id': target2 or target,
        'transform_space': space2 + '_SPACE',
    }

    if bone2 is not None:
        target2_map['bone_target'] = bone2

    return { 'type': 'LOC_DIFF', 'targets': [ target1_map, target2_map ] }


def create_sample(obj):
    from rigify.rigs.basic.copy_chain import create_sample as inner
    obj.pose.bones[inner(obj)["bone.01"]].rigify_type = 'skin.basic_chain'
