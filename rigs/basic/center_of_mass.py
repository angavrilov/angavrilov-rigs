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
import bmesh
import collections

from bmesh.types import BMFace

from itertools import count
from mathutils import Matrix, Vector

from bl_math import clamp

from rigify.utils.bones import put_bone
from rigify.utils.naming import org, strip_prefix, make_derived_name
from rigify.utils.widgets import widget_generator

from rigify.base_rig import stage, BaseRig
from rigify.base_generate import GeneratorPlugin


class Rig(BaseRig):
    """
    Generates a bone that tracks the center of mass of the character.
    """

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        self.cage_obj = self.params.com_volume_cage
        if not self.cage_obj:
            self.raise_error('Cage object is not specified')

        # Clear the reference
        self.params.com_volume_cage = None

        self.com_table = calc_vgroup_com(self, self.cage_obj)

        if not self.com_table:
            self.raise_error('Cage does not specify any bones')

        PostGenerateCaller(self.generator).add_rig(self)

    ##############################
    # BONES
    #
    # org:
    #   the bone
    # ctrl:
    #   master:
    #     the visible 'control'
    # mch:
    #   helpers[]
    #     helper mch bones
    #
    ##############################

    ##############################
    # Display bone

    @stage.generate_bones
    def make_master_control(self):
        org = self.bones.org
        self.bones.ctrl.master = self.copy_bone(org, make_derived_name(org, 'ctrl'))

    @stage.parent_bones
    def parent_master_control(self):
        self.set_bone_parent(self.bones.ctrl.master, self.bones.org)

    @stage.configure_bones
    def configure_master_control(self):
        bone = self.get_bone(self.bones.ctrl.master)
        bone.lock_location = True, True, True
        bone.lock_rotation = True, True, True
        bone.lock_rotation_w = True
        bone.lock_scale = True, True, True

    @stage.generate_widgets
    def generate_master_widget(self):
        create_com_widget(self.obj, self.bones.ctrl.master, radius=4)

    ##############################
    # Mechanism

    def post_generate_bones(self):
        eb = self.obj.data.edit_bones

        max_error = self.params.com_precision
        tot_mass = sum(mass for mass, _ in self.com_table.values())

        # Find head/tail projections and its error
        bone_list = []

        for name, (mass, com) in self.com_table.items():
            if name not in eb:
                self.raise_error(f'Cannot find bone: {name}')

            bone = eb[name]
            mat = bone.matrix
            length = bone.length

            com_local = mat.inverted() @ com
            com_local_clip = Vector((0, clamp(com_local.y, 0, length), 0))

            head_tail = com_local_clip.y / length
            error = (com_local - com_local_clip).length * mass / tot_mass

            bone_list.append((name, mass / tot_mass, head_tail, error, com))

        tot_error = sum(t[3] for t in bone_list)

        # Generate helper bones to get error under the threshold
        bone_list.sort(reverse=True, key=lambda t: t[3])

        final_list = []
        helpers = []

        org = self.bones.org

        for name, mass, head_tail, error, com in bone_list:
            if tot_error > max_error:
                tot_error -= error

                mch = make_derived_name(org, 'mch', '.' + strip_prefix(name))
                mch = self.copy_bone(org, mch, scale=0.1)
                put_bone(self.obj, mch, com)

                helpers.append(mch)
                final_list.append((name, mch, mass, 0))
            else:
                final_list.append((name, name, mass, head_tail))

        self.bone_mapping = final_list
        self.bones.mch.helpers = helpers

    @stage.parent_bones
    def parent_mch_helpers(self):
        for name, mch, mass, head_tail in self.bone_mapping:
            if name != mch:
                self.set_bone_parent(mch, name)

    @stage.rig_bones
    def rig_org_bone(self):
        org = self.bones.org
        total = 0

        for name, mch, mass, head_tail in self.bone_mapping:
            total += mass
            self.make_constraint(
                org, 'COPY_LOCATION', mch,
                influence=mass/total,
                head_tail=head_tail,
            )

    ##############################
    # UI

    @classmethod
    def add_parameters(self, params):
        params.com_volume_cage = bpy.props.PointerProperty(
            type=bpy.types.Object, name='Volume Cage Mesh',
            description='Mesh object used to calculate mass distribution: should contain a separate manifold submesh assigned to each relevant bone vertex group',
            poll=lambda self, obj: obj.type == 'MESH'
        )

        params.com_precision = bpy.props.FloatProperty(
            name='Maximum Error', min=0, default=0.01,
            description='Maximum positional error allowed if necessary to reduce rig complexity',
            subtype='DISTANCE',
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, 'com_precision')

        layout.label(text='Volume Cage Mesh:')
        layout.prop(params, 'com_volume_cage', text='')

        row = layout.row()
        row.enabled = not params.com_volume_cage
        row.operator('mesh.rigify_add_com_volume_cage', text='Add Sample Cage')


class PostGenerateCaller(GeneratorPlugin):
    priority = -50

    def __init__(self, generator):
        super().__init__(generator)

        self.rig_list = []

    def add_rig(self, rig):
        assert self.generator.stage == 'initialize'
        self.rig_list.append(rig)

    def generate_bones(self):
        for rig in self.rig_list:
            rig.post_generate_bones()


@widget_generator
def create_com_widget(geom, *, radius=0.5):
    """Creates a widget similar to Plain Axes empty, but with a cross or
       a square on the end of each axis line.
    """
    axis = radius
    geom.verts = [(axis, 0, 0), (-axis, 0, 0), (0, axis, 0),
                  (0, -axis, 0), (0, 0, -axis), (0, 0, axis)]
    geom.edges = [(0, 1), (2, 3), (4, 5)]


def create_sample(obj):
    """ Create a sample metarig for this rig type.
    """
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('Bone')
    bone.head[:] = 0.0000, 0.0000, 0.0000
    bone.tail[:] = 0.0000, 0.5000, 0.0000
    bone.roll = 0.0000
    bone.use_connect = False
    bones['Bone'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['Bone']]
    pbone.rigify_type = 'basic.center_of_mass'
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
        arm.edit_bones.active = bone

    return bones


def split_vgroup_components(rig, obj):
    groups = {i: vg.name for i, vg in enumerate(obj.vertex_groups)}

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.index_update()

    # Sort verts into groups
    deform = bm.verts.layers.deform.verify()

    vert_vg = {}
    vg_verts = collections.defaultdict(list)

    for vert in bm.verts:
        vgs = [vgid for vgid, weight in vert[deform].items() if weight > 0]

        if len(vgs) > 1:
            names = ', '.join(groups.get(i, str(i)) for i in vgs)
            rig.raise_error(f'Cage vertex assigned to multiple groups: {names}')

        if vgs:
            vert_vg[vert.index] = vgs[0]
            vg_verts[vgs[0]].append(vert)

    # Sort faces into groups
    vg_faces = collections.defaultdict(list)

    for face in bm.faces:
        vgs = list(set(vert_vg.get(vert.index, -1) for vert in face.verts))

        if len(vgs) > 1:
            names = ', '.join(groups.get(i, str(i)) for i in vgs)
            rig.raise_error(f'Cage face assigned to multiple groups: {names}')

        if vgs:
            vg_faces[vgs[0]].append(face)

    # Extract faces for group components
    comp_table = {}

    for gid, gname in groups.items():
        if gid not in vg_faces:
            rig.raise_error(f'No faces assigned to group: {gname}')

        rv = bmesh.ops.split(bm, geom=vg_faces[gid], use_only_faces=True)

        comp_table[gname] = rv['geom']

    return bm, comp_table


def calc_mesh_center_of_mass(rig, bm, geom, name):
    faces = []

    for item in geom:
        if isinstance(item, BMFace):
            faces.append(item)
        elif not item.is_manifold:
            rig.raise_error(f'Mesh component {name} is not manifold')

    tri_rv = bmesh.ops.triangulate(bm, faces=faces)

    center = sum((v.co for v in bm.verts), Vector((0, 0, 0))) / len(bm.verts)

    acc_volume = 0
    acc_com = Vector((0, 0, 0))

    for face in tri_rv['faces']:
        p1, p2, p3 = [l.vert.co - center for l in face.loops]

        volume = Matrix((p1, p2, p3)).determinant() / 6
        com = (p1 + p2 + p3) / 4

        acc_volume += volume
        acc_com += com * volume

    return acc_volume, center + (acc_com / acc_volume)


def calc_vgroup_com(rig, obj):
    bm, comp_table = split_vgroup_components(rig, obj)

    return {
        name: calc_mesh_center_of_mass(rig, bm, geom, name)
        for name, geom in comp_table.items()
    }


class MESH_OT_rigify_add_com_volume_cage(bpy.types.Operator):
    bl_idname = 'mesh.rigify_add_com_volume_cage'
    bl_label = "Add Center Of Mass Volume Cage"
    bl_description = "Add a cage mesh for the Rigify center of mass rig"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    @classmethod
    def poll(cls, context):
        return context.object and context.active_pose_bone

    SIZE_TABLE = {
        'spine': (0.5, 0.5, 1, 0.5),
        'spine.001': (1, 0.5, 1, 0.5),
        'spine.002': (1, 0.5, 1, 0.5),
        'spine.003': (1, 0.5, 1, 0.5),
        'spine.004': (0.7, 0.7, 0.7, 0.7),
        'spine.005': (0.7, 0.7, 0.7, 0.7),
        'spine.006': (0.5, 0.5, 0.5, 0.5),
        'head': (0.5, 0.5, 0.5, 0.5),
        'hand.L': (0.2, 0.5, 0.2, 0.4),
        'hand.R': (0.2, 0.5, 0.2, 0.4),
        'foot.L': (0.4, 0.3, 0.4, 0.3),
        'foot.R': (0.4, 0.3, 0.4, 0.3),
    }

    def generate_bone_cage(self, vertices, faces, vgroups, bone):
        vbase = len(vertices)
        mat = bone.bone.matrix_local @ Matrix.Scale(bone.length, 4)

        size = self.SIZE_TABLE.get(bone.name, (0.15, 0.15, 0.1, 0.1))
        rx_head, rz_head, rx_tail, rz_tail = size

        vertices += [
            mat @ Vector(vert) for vert in [
                (-rx_head, 0, -rz_head),
                (rx_head, 0, -rz_head),
                (rx_head, 0, rz_head),
                (-rx_head, 0, rz_head),
                (-rx_tail, 1, -rz_tail),
                (rx_tail, 1, -rz_tail),
                (rx_tail, 1, rz_tail),
                (-rx_tail, 1, rz_tail),
            ]
        ]

        faces += [
            tuple(idx + vbase for idx in tup) for tup in [
                (0, 1, 2, 3), (1, 5, 6, 2), (5, 4, 7, 6),
                (4, 0, 3, 7), (4, 5, 1, 0), (3, 2, 6, 7),
            ]
        ]

        name = org(bone.name)

        vgroups.append((name, [i + vbase for i in range(8)]))

    def create_mesh(self, obj, mesh, bones):
        vertices = []
        faces = []
        vgroups = []

        for bone in bones:
            self.generate_bone_cage(vertices, faces, vgroups, bone)

        mesh.from_pydata(vertices, [], faces)
        mesh.update()

        for vgname, verts in vgroups:
            vg = obj.vertex_groups.new(name=vgname)
            vg.add(verts, 1.0, 'REPLACE')

    def execute(self, context):
        pbone = context.active_pose_bone
        name = 'CAGE-' + pbone.name

        bones = [
            bone for bone in context.selected_pose_bones if bone != pbone
        ]

        if not bones:
            self.report({'ERROR'}, 'Select bones to include in the COM')
            return {'CANCELLED'}

        mesh = bpy.data.meshes.new(name)
        obj = bpy.data.objects.new(name, mesh)

        self.create_mesh(obj, mesh, bones)

        obj.matrix_world = context.active_object.matrix_world
        obj.display_type = 'WIRE'
        obj.hide_render = True

        context.collection.objects.link(obj)
        pbone.rigify_parameters.com_volume_cage = obj

        return {'FINISHED'}
