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

from rigify.utils.naming import make_derived_name, mirror_name, change_name_side, Side, SideZ
from rigify.utils.bones import align_bone_z_axis, put_bone
from rigify.utils.widgets import create_widget
from rigify.utils.widgets_basic import create_circle_widget
from rigify.utils.switch_parent import SwitchParentBuilder
from rigify.utils.misc import map_list, matrix_from_axis_pair

from rigify.base_rig import stage, RigComponent

from ..skin.skin_rigs import BaseSkinRig, ControlBoneNode, ControlBoneParentOffset, LazyRef
from ..skin.basic_chain import Rig as BasicChainRig

import functools
import mathutils

from itertools import count
from mathutils import Vector, Matrix


class Rig(BaseSkinRig):
    """Eye rig."""

    def find_org_bones(self, bone):
        return bone.name

    cluster_control = None

    def initialize(self):
        super().initialize()

        bone = self.get_bone(self.base_bone)
        self.center = bone.head
        self.axis = bone.vector

        self.eye_corner_nodes = []
        self.eye_corner_matrix = None

        if not self.cluster_control:
            self.cluster_control = self.create_cluster_control()

        self.init_child_chains()


    ####################################################
    # Utilities

    def init_child_chains(self):
        self.child_chains = [ rig for rig in self.rigify_children if isinstance(rig, BasicChainRig) ]

        for child in self.child_chains:
            self.patch_chain(child)

    def patch_chain(self, child):
        return ChainPatch(child, self)

    def create_cluster_control(self):
        return EyeClusterControl(self)

    def init_eye_corner_space(self):
        if self.eye_corner_matrix:
            return

        if len(self.eye_corner_nodes) != 2:
            self.raise_error('Expected 2 eye corners, but found {}', len(self.eye_corner_nodes))

        vecs = [ (node.point - self.center).normalized() for node in self.eye_corner_nodes ]
        normal = vecs[0].cross(vecs[1])
        space_axis = self.axis - self.axis.project(normal)

        matrix = matrix_from_axis_pair(space_axis, normal, 'z').to_4x4()
        matrix.translation = self.center
        self.eye_corner_matrix = matrix.inverted()

        amin, amax = self.eye_corner_range = list(sorted(map(self.get_eye_corner_angle, self.eye_corner_nodes)))

        if not (amin <= 0 <= amax):
            self.raise_error('Bad relative angles of eye corners: {}..{}', math.degrees(amin), math.degrees(amax))

    def get_eye_corner_angle(self, node):
        pt = self.eye_corner_matrix @ node.point
        return math.atan2(pt.x, pt.y)

    def get_master_control_position(self):
        self.init_eye_corner_space()

        pcorners = [ node.point for node in self.eye_corner_nodes ]

        point, _ = mathutils.geometry.intersect_line_line(
            self.center, self.center + self.axis, pcorners[0], pcorners[1]
        )
        return point

    def get_lid_follow_influence(self, node):
        self.init_eye_corner_space()

        angle = self.get_eye_corner_angle(node)
        amin, amax = self.eye_corner_range

        if amin < angle < 0:
            return 1 - min(1, angle/amin) ** 2
        elif 0 < angle < amax:
            return 1 - min(1, angle/amax) ** 2
        else:
            return 0

    ####################################################
    # Control nodes

    def is_eye_control_node(self, node):
        return node.rig in self.child_chains and node.is_master_node

    def is_eye_corner_node(self, node):
        sides = set(n.name_split[2] for n in node.get_merged_siblings())
        return {SideZ.BOTTOM, SideZ.TOP}.issubset(sides)

    def extend_control_node_parent(self, parent, node):
        if self.is_eye_control_node(node):
            if self.is_eye_corner_node(node):
                self.eye_corner_nodes.append(node)
            else:
                return self.extend_mid_node_parent(parent, node)

        return parent

    def extend_mid_node_parent(self, parent, node):
        parent = ControlBoneParentOffset.wrap(self, parent, node)
        parent.add_copy_local_location(
            LazyRef(self.bones.mch, 'track'),
            influence=LazyRef(self.get_lid_follow_influence, node)
        )
        return parent

    def extend_control_node_rig(self, node):
        if self.is_eye_control_node(node):
            self.make_constraint(
                node.control_bone, 'LIMIT_DISTANCE', self.bones.org,
                distance = (node.point - self.center).length,
                limit_mode='LIMITDIST_ONSURFACE', use_transform_limit=True,
                # Use custom space to accomodate scaling
                space='CUSTOM', space_object=self.obj, space_subtarget=self.bones.org,
            )

    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control(self):
        org = self.bones.org
        name = self.copy_bone(org, make_derived_name(org, 'ctrl'), parent=True)
        put_bone(self.obj, name, self.get_master_control_position())
        self.bones.ctrl.master = name

    @stage.configure_bones
    def configure_master_control(self):
        self.copy_bone_properties(self.bones.org, self.bones.ctrl.master)

    @stage.generate_widgets
    def make_master_control_widget(self):
        ctrl = self.bones.ctrl.master
        create_circle_widget(self.obj, ctrl)


    ####################################################
    # Tracking MCH

    @stage.generate_bones
    def make_mch_track_bones(self):
        org = self.bones.org
        mch = self.bones.mch

        mch.master = self.copy_bone(org, make_derived_name(org, 'mch'))
        mch.track = self.copy_bone(org, make_derived_name(org, 'mch', '_track'), scale=1/4)

        put_bone(self.obj, mch.track, self.get_bone(org).tail)

    @stage.parent_bones
    def parent_mch_track_bones(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl
        self.set_bone_parent(mch.master, ctrl.master)
        self.set_bone_parent(mch.track, ctrl.master)

    @stage.configure_bones
    def configure_mch_track_bones(self):
        ctrl = self.bones.ctrl

        controls = sum((chain.bones.ctrl.flatten() for chain in self.child_chains), ctrl.flatten())
        panel = self.script.panel_with_selected_check(self, controls)

        self.make_property(ctrl.target, 'lid_follow', 1.0, description='Eylids follow eye movement')
        panel.custom_prop(ctrl.target, 'lid_follow', text='Eyelids Follow', slider=True)

    @stage.rig_bones
    def rig_mch_track_bones(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl

        self.make_constraint(mch.master, 'DAMPED_TRACK', ctrl.target)

        con = self.make_constraint(mch.track, 'COPY_LOCATION', mch.master, head_tail=1)
        self.make_driver(con, 'influence', variables=[(ctrl.target, 'lid_follow')])


    ####################################################
    # ORG bone

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org, self.bones.ctrl.master)


    ####################################################
    # Deform bones

    @stage.generate_bones
    def make_deform_bone(self):
        org = self.bones.org
        deform = self.bones.deform
        deform.master = self.copy_bone(org, make_derived_name(org, 'def', '_master'), scale=3/2)

        if self.params.make_deform:
            deform.eye = self.copy_bone(org, make_derived_name(org, 'def'))
            deform.iris = self.copy_bone(org, make_derived_name(org, 'def', '_iris'), scale=1/2)
            put_bone(self.obj, deform.iris, self.get_bone(org).tail)

    @stage.parent_bones
    def parent_deform_chain(self):
        deform = self.bones.deform
        self.set_bone_parent(deform.master, self.bones.org)

        if self.params.make_deform:
            self.set_bone_parent(deform.eye, self.bones.mch.master)
            self.set_bone_parent(deform.iris, deform.eye)

    @stage.rig_bones
    def rig_deform_chain(self):
        if self.params.make_deform:
            self.make_constraint(
                self.bones.deform.iris, 'COPY_SCALE', self.bones.ctrl.target,
                owner_space='LOCAL', target_space='OWNER_LOCAL', use_y=False,
            )


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.make_deform = bpy.props.BoolProperty(
            name        = "Deform",
            default     = True,
            description = "Create a deform bone for the copy"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        r = layout.row()
        r.prop(params, "make_deform", text="Eyball And Iris Deforms")


class ChainPatch(RigComponent):
    "Twist handles to aim Z axis at the eye center"

    rigify_sub_object_run_late = True

    def __init__(self, owner, eye):
        super().__init__(owner)

        self.eye = eye
        self.owner.use_pre_handles = True

    def align_bone(self, name):
        align_bone_z_axis(self.obj, name, self.eye.center - self.get_bone(name).head)

    def prepare_bones(self):
        for org in self.owner.bones.org:
            self.align_bone(org)

    def generate_bones(self):
        if self.owner.use_bbones:
            for pre in self.owner.bones.mch.handles_pre:
                self.align_bone(pre)

    def rig_bones(self):
        if self.owner.use_bbones:
            for pre, node in zip(self.owner.bones.mch.handles_pre, self.owner.control_nodes):
                self.make_constraint(pre, 'COPY_LOCATION', node.control_bone, name='locate_cur')
                self.make_constraint(
                    pre, 'LOCKED_TRACK', self.eye.bones.org, name='track_center',
                    track_axis='TRACK_Z', lock_axis='LOCK_Y',
                )


class EyeClusterControl(RigComponent):
    "Creates a common control for an eye cluster"

    def __init__(self, owner):
        super().__init__(owner)

        self.find_cluster_rigs()

    def find_cluster_rigs(self):
        owner = self.owner
        parent_rig = owner.rigify_parent

        owner.cluster_control = self
        self.rig_list = [ owner ]

        if parent_rig:
            for rig in parent_rig.rigify_children:
                if isinstance(rig, Rig) and rig != owner:
                    rig.cluster_control = self
                    self.rig_list.append(rig)

        self.rig_count = len(self.rig_list)

    def find_cluster_position(self):
        bone = self.get_bone(self.owner.base_bone)

        axis = Vector((0,0,0))
        center = Vector((0,0,0))
        length = 0

        for rig in self.rig_list:
            bone = self.get_bone(rig.base_bone)
            axis += bone.y_axis
            center += bone.head
            length += bone.length

        axis /= self.rig_count
        center /= self.rig_count
        length /= self.rig_count

        matrix = matrix_from_axis_pair((0,0,1), axis, 'z').to_4x4()
        matrix.translation = center + axis * length * 5

        self.size = length * 3 / 4
        self.matrix = matrix
        self.inv_matrix = matrix.inverted()

    def project_rig_control(self, rig):
        bone = self.get_bone(rig.base_bone)

        head = self.inv_matrix @ bone.head
        tail = self.inv_matrix @ bone.tail
        axis = tail - head

        return head + axis * (-head.z / axis.z)

    def get_common_rig_name(self):
        names = set(rig.base_bone for rig in self.rig_list)
        name = min(names)

        if mirror_name(name) in names:
            return change_name_side(name, side=Side.MIDDLE)

        return name

    def get_rig_control_matrix(self, rig):
        matrix = self.matrix.copy()
        matrix.translation = self.matrix @ self.rig_points[rig]
        return matrix

    def initialize(self):
        self.find_cluster_position()
        self.rig_points = { rig: self.project_rig_control(rig) for rig in self.rig_list }

    def generate_bones(self):
        if self.rig_count > 1:
            self.master_bone = self.make_master_control()
            self.child_bones = []

            for rig in self.rig_list:
                rig.bones.ctrl.target = child = self.make_child_control(rig)
                self.child_bones.append(child)
        else:
            self.master_bone = self.make_child_control(self.rig_list[0])
            self.child_bones = [ self.master_bone ]
            self.owner.bones.ctrl.target = self.master_bone

        self.build_parent_switch()

    def make_master_control(self):
        name = self.new_bone(make_derived_name(self.get_common_rig_name(), 'ctrl', '_common'))
        bone = self.get_bone(name)
        bone.matrix = self.matrix
        bone.length = self.size
        return name

    def make_child_control(self, rig):
        name = rig.copy_bone(rig.base_bone, make_derived_name(rig.base_bone, 'ctrl'), length=self.size)
        self.get_bone(name).matrix = self.get_rig_control_matrix(rig)
        return name

    def build_parent_switch(self):
        pbuilder = SwitchParentBuilder(self.owner.generator)

        org_parent = self.owner.rig_parent_bone
        parents = [org_parent] if org_parent else []

        pbuilder.build_child(
            self.owner, self.master_bone,
            prop_name="Parent ({})".format(self.master_bone),
            extra_parents=[self.owner.rig_parent_bone], select_parent=org_parent,
            controls=self.get_all_rig_control_bones
        )

    def get_all_rig_control_bones(self):
        return list(set(sum((rig.bones.ctrl.flatten() for rig in self.rig_list), [self.master_bone])))

    def parent_bones(self):
        if self.rig_count > 1:
            for child in self.child_bones:
                self.set_bone_parent(child, self.master_bone)

    def configure_bones(self):
        for child in self.child_bones:
            bone = self.get_bone(child)
            bone.lock_rotation = (True,True,True)
            bone.lock_rotation_w = True

    def generate_widgets(self):
        for child in self.child_bones:
            create_eye_widget(self.obj, child)

        if self.rig_count > 1:
            pt2d = [p.to_2d() / self.size for p in self.rig_points.values()]
            create_eyes_widget(self.obj, self.master_bone, points=pt2d)


def widget_generator(generate_func):
    """
    Decorator that encapsulates a call to create_widget, and only requires
    the actual function to fill the provided vertex and edge lists.
    """
    @functools.wraps(generate_func)
    def wrapper(rig, bone_name, bone_transform_name=None, **kwargs):
        obj = create_widget(rig, bone_name, bone_transform_name)
        if obj is not None:
            verts = []
            edges = []

            generate_func(verts, edges, **kwargs)

            mesh = obj.data
            mesh.from_pydata(verts, edges, [])
            mesh.update()
            return obj
        else:
            return None

    return wrapper

def generate_circle_geometry(verts, edges, center, radius, *, matrix=None, angle_range=None, steps=24):
    """
    Generates a circle, adding vertices and edges to the lists.
    center, radius: parameters of the circle
    matrix: transformation matrix (by default the circle is in the XY plane)
    angle_range: pair of angles to generate an arc of the circle
    steps: number of edges to cover the whole circle (reduced for arcs)
    """
    base = len(verts)
    start = 0
    delta = math.pi * 2 / steps

    if angle_range:
        start, end = angle_range
        if start >= end:
            return False

        steps = max(3, math.ceil((end - start) / delta) + 1)
        delta = (end - start) / (steps - 1)
    else:
        start = 0
        end = math.pi * 2

    for i in range(steps):
        angle = start + delta * i
        point = center + Vector((math.cos(angle), math.sin(angle), 0)) * radius

        if matrix:
            point = matrix @ point

        verts.append(point)
        if i > 0:
            edges.append((base + i - 1, base + i))

    if not angle_range:
        edges.append((base + steps - 1, base))

    return True

@widget_generator
def create_eye_widget(verts, edges, *, size=1):
    generate_circle_geometry(verts, edges, Vector((0,0,0)), size/2)

def generate_circle_hull_geometry(verts, edges, points, radius, gap, *, matrix=None, steps=24):
    """
    Given a list of 2D points forming a convex hull, generate a contour around
    it, with each point being circumscribed with a circle arc of given radius,
    and keeping the given distance gap from the lines connecting the circles.
    """
    base = len(verts)
    points_ex = [points[-1], *points, points[0]]
    agap = math.asin(gap / radius)

    for i, pprev, pcur, pnext in zip(count(0), points_ex[0:], points_ex[1:], points_ex[2:]):
        vprev = pprev - pcur
        vnext = pnext - pcur

        # Compute bearings to adjacent points
        aprev = math.atan2(vprev.y, vprev.x)
        anext = math.atan2(vnext.y, vnext.x)
        if anext <= aprev:
            anext += math.pi * 2

        # Adjust gap for circles that are too close
        aprev += max(agap, math.acos(min(1, vprev.length/radius/2)))
        anext -= max(agap, math.acos(min(1, vnext.length/radius/2)))

        if anext > aprev:
            if len(verts) > base:
                edges.append((len(verts)-1, len(verts)))

            generate_circle_geometry(
                verts, edges, pcur.to_3d(), radius,
                angle_range=(aprev, anext),
                matrix=matrix, steps=steps
            )

    if len(verts) > base:
        edges.append((len(verts)-1, base))

@widget_generator
def create_eyes_widget(verts, edges, *, size=1, points):
    hpoints = [points[i] for i in mathutils.geometry.convex_hull_2d(points)]

    generate_circle_hull_geometry(verts, edges, hpoints, size*3/4, size/2)
    generate_circle_hull_geometry(verts, edges, hpoints, size, size*3/4)
