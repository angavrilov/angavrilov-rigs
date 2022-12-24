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
import math

from itertools import repeat

from rigify.utils.bones import put_bone
from rigify.utils.mechanism import make_constraint, make_driver, make_property
from rigify.utils.mechanism import deactivate_custom_properties, reactivate_custom_properties,\
    copy_custom_properties_with_ui
from rigify.utils.naming import make_derived_name
from rigify.utils.misc import map_list

from rigify.base_rig import stage

from . import basic


class Rig(basic.Rig):
    """
    A version of basic jiggle with support for a cloth simulation cage
    that is used to deform part of the final mesh via Surface Deform.

    Custom properties on the cage object and mesh that have names starting
    with 'option_' are automatically copied to the rig bone and linked
    with drivers. Anchor empties parented to the cage can be used to
    feed the result of cloth simulation and/or cage shape keys to the rig.

    Resetting all custom properties on the cage object and mesh to defaults,
    and disabling the Armature modifier must always reconfigure it and the
    anchors into the rest shape that the rig should bind to. The cage can only
    depend on the first deform bone of this rig.
    """

    cage_obj: bpy.types.Object

    use_shape_anchor: bool
    use_shape_only_location: bool
    use_front_anchor: bool

    def initialize(self):
        super().initialize()

        self.cage_obj = self.params.jiggle_cloth_cage
        if not self.cage_obj:
            self.raise_error('Physics cage object is not specified')

        self.use_shape_anchor = self.params.jiggle_shape_anchor is not None
        self.use_shape_only_location = self.params.jiggle_shape_only_location
        self.use_front_anchor = \
            len(self.bones.org) > 1 and self.params.jiggle_front_anchor is not None

    ##############################
    # Cage

    @stage.prepare_bones
    def prepare_cage(self):
        deactivate_custom_properties(self.cage_obj)
        deactivate_custom_properties(self.cage_obj.data)

        for mod in self.cage_obj.modifiers:
            if mod.type == 'ARMATURE':
                mod.show_render = mod.show_viewport = False

    @stage.configure_bones
    def configure_cage(self):
        bone_name = self.bones.ctrl.master if self.use_master_control else self.bones.ctrl.front

        copy_custom_properties_with_ui(
            self, self.cage_obj, bone_name, prefix='option_', link_driver=True)
        copy_custom_properties_with_ui(
            self, self.cage_obj.data, bone_name, prefix='option_', link_driver=True)

    @stage.finalize
    def finalize_cage(self):
        reactivate_custom_properties(self.cage_obj)
        reactivate_custom_properties(self.cage_obj.data)

        for mod in self.cage_obj.modifiers:
            if mod.type == 'ARMATURE':
                mod.object = self.obj
                mod.show_render = mod.show_viewport = True

    ##############################
    # BONES

    class MchBones(basic.Rig.MchBones):
        shape_anchor: list[str]        # Shape anchor system

    bones: basic.Rig.ToplevelBones[
        list[str],
        'Rig.CtrlBones',
        'Rig.MchBones',
        list[str]
    ]

    ##############################
    # Shape anchor

    @stage.generate_bones
    def make_mch_shape_anchor(self):
        if self.use_shape_anchor:
            org = self.bones.org[0]
            if self.use_shape_only_location:
                self.bones.mch.shape_anchor = \
                    map_list(self.make_mch_shape_anchor_bone, range(2), repeat(org))
            else:
                self.bones.mch.shape_anchor = \
                    map_list(self.make_mch_shape_anchor_bone, range(4), repeat(org))

    def make_mch_shape_anchor_bone(self, i: int, org: str):
        name = self.copy_bone(org, make_derived_name(org, 'mch', '_shape'+str(i)))

        if i == 0 and self.use_shape_only_location:
            pos = self.params.jiggle_shape_anchor.matrix_world.translation
            put_bone(self.obj, name, self.obj.matrix_world.inverted() @ pos)

        return name

    @stage.parent_bones
    def parent_mch_shape_anchor(self):
        if self.use_shape_anchor:
            chain = self.bones.mch.shape_anchor
            self.generator.disable_auto_parent(chain[0])
            if not self.use_shape_only_location:
                self.generator.disable_auto_parent(chain[1])
                self.set_bone_parent(chain[2], chain[1])
            self.set_bone_parent(chain[-1], self.rig_parent_bone)

    @stage.rig_bones
    def rig_mch_shape_anchor(self):
        if self.use_shape_anchor:
            chain = self.bones.mch.shape_anchor
            anchor = self.params.jiggle_shape_anchor
            deform = self.bones.deform[0]

            if self.use_shape_only_location:
                anchor_bone, offset_bone = chain

                make_constraint(self.get_bone(anchor_bone), 'COPY_LOCATION', anchor, space='WORLD')

                self.make_constraint(offset_bone, 'COPY_LOCATION', anchor_bone, space='LOCAL')
                self.make_constraint(deform, 'COPY_LOCATION', anchor_bone,
                                     invert_xyz=(True, True, True), space='LOCAL')
            else:
                rest_bone, anchor_bone, invert_bone, offset_bone = chain

                make_constraint(self.get_bone(anchor_bone), 'CHILD_OF', anchor,
                                inverse_matrix=anchor.matrix_world.inverted())

                self.make_constraint(invert_bone, 'COPY_TRANSFORMS', rest_bone, space='POSE')
                self.make_constraint(offset_bone, 'COPY_TRANSFORMS', anchor_bone, space='LOCAL')

                self.make_constraint(deform, 'COPY_TRANSFORMS', invert_bone, space='LOCAL')

    def get_master_parent(self):
        if self.use_shape_anchor:
            return self.bones.mch.shape_anchor[-1]
        else:
            return self.rig_parent_bone

    ##############################
    # Front anchor

    @stage.parent_bones
    def parent_org_chain(self):
        super().parent_org_chain()

        if self.use_front_anchor:
            self.set_bone_parent(self.bones.org[1], None)
            self.generator.disable_auto_parent(self.bones.org[1])

    def rig_front_org_bone(self, org: str):
        if self.use_front_anchor:
            anchor = self.params.jiggle_front_anchor

            make_constraint(self.get_bone(org), 'CHILD_OF', anchor,
                            inverse_matrix=anchor.matrix_world.inverted())
        else:
            super().rig_front_org_bone(org)

    ##############################
    # UI

    @classmethod
    def add_parameters(cls, params):
        super().add_parameters(params)

        params.jiggle_cloth_cage = bpy.props.PointerProperty(
            type=bpy.types.Object, name='Cloth Cage Mesh',
            description='Mesh object used for cloth simulation',
            poll=lambda self, obj: obj.type == 'MESH'
        )

        params.jiggle_front_anchor = bpy.props.PointerProperty(
            type=bpy.types.Object, name='Front Anchor',
            description='Anchor object parented to the front of the cage',
        )

        params.jiggle_shape_anchor = bpy.props.PointerProperty(
            type=bpy.types.Object, name='Shape Anchor',
            description='Anchor used to move the rig to match the shape keys of the cage',
        )

        params.jiggle_shape_only_location = bpy.props.BoolProperty(
            default=True, name='Only Use Shape Anchor Location',
            description='Only location of the shape anchor is used',
        )

    @classmethod
    def parameters_ui(cls, layout, params):
        super().parameters_ui(layout, params)

        layout.label(text='Cloth Cage Mesh:')
        layout.prop(params, 'jiggle_cloth_cage', text='')
        layout.prop(params, 'jiggle_front_anchor')

        row = layout.row()
        row.enabled = not params.jiggle_cloth_cage
        row.operator('mesh.rigify_add_jiggle_cloth_cage', text='Add Sample Cage')

        layout.prop(params, 'jiggle_shape_anchor')

        row = layout.row()
        row.active = not not params.jiggle_shape_anchor
        row.prop(params, 'jiggle_shape_only_location')

        row = layout.row()
        row.enabled = not params.jiggle_shape_anchor
        row.operator('mesh.rigify_add_jiggle_shapekey_anchor')


def create_sample(obj):
    """ Create a sample metarig for this rig type.
    """
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('Bone')
    bone.head[:] = 0.0000, 0.0000, 0.0000
    bone.tail[:] = 0.0000, -0.1000, 0.0000
    bone.roll = 0.0000
    bone.use_connect = False
    bones['Bone'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['Bone']]
    pbone.rigify_type = 'jiggle.cloth_cage'
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


# noinspection PyPep8Naming
class MESH_OT_rigify_add_jiggle_cloth_cage(bpy.types.Operator):
    bl_idname = 'mesh.rigify_add_jiggle_cloth_cage'
    bl_label = "Add Cloth Cage"
    bl_description = "Add a sample cloth cage mesh object"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    @classmethod
    def poll(cls, context):
        return context.object and context.active_pose_bone

    base_start_idx: int
    row_size: int
    row_cnt: int
    vertex_count: int

    def create_mesh_data(self, mesh: bpy.types.Mesh, radius: float, steps_x: int, steps_y: int):
        vertices = [(0, radius, 0)]
        steps_x_4 = steps_x * 4
        steps_y_2 = int(steps_y / 2)

        self.base_start_idx = 1 + (steps_y - 1) * steps_x_4
        self.row_size = steps_x_4
        self.row_cnt = steps_y

        for i in range(steps_y):
            for j in range(steps_x_4):
                delta = ((j % 2) - 0.5) * 0.1 if i == 0 else 0
                alpha = (i + 1 + delta) * math.pi / 2 / steps_y
                rx = math.sin(alpha) * radius
                y = math.cos(alpha) * radius
                beta = j * math.pi / 2 / steps_x
                x = math.sin(beta) * rx
                z = math.cos(beta) * rx
                vertices.append((x, y, z))

        for i in range(steps_y_2-1):
            rx = radius * (steps_y_2-1-i) / steps_y_2
            for j in range(steps_x_4):
                beta = j * math.pi / 2 / steps_x
                x = math.sin(beta) * rx
                z = math.cos(beta) * rx
                vertices.append((x, 0, z))

        last_idx = len(vertices)
        vertices.append((0, 0, 0))

        self.vertex_count = len(vertices)

        faces = []

        for j in range(0, steps_x_4, 2):
            faces.append((0, j+1, j+2, 1+(j+2) % steps_x_4))
            faces.append((last_idx, last_idx-1-j, last_idx-2-j, last_idx-1-(j+2) % steps_x_4))

        for i in range(0, steps_y + steps_y_2 - 2):
            for j in range(steps_x_4):
                j1 = (j+1) % steps_x_4
                faces.append((1 + i*steps_x_4 + j, 1 + (i+1)*steps_x_4 + j,
                              1 + (i+1)*steps_x_4 + j1, 1 + i*steps_x_4 + j1))

        mesh.from_pydata(vertices, [], faces)
        mesh.update()

    @staticmethod
    def make_up_shape_key(obj: bpy.types.Object):
        sk = obj.shape_key_add(name='Up')

        make_driver(sk, 'value', variables=[(obj, obj, 'option_up')])

        for item in sk.data:
            item.co[2] += item.co[1] * 0.2

    @staticmethod
    def add_cloth_sim(obj: bpy.types.Object, size: float):
        mod = obj.modifiers.new(name='Cloth', type='CLOTH')

        assert isinstance(mod, bpy.types.ClothModifier)

        make_driver(mod, 'show_viewport', variables=[(obj, obj, 'option_physics')])
        make_driver(mod, 'show_render', variables=[(obj, obj, 'option_physics')])

        k = 0.1 / size

        cs = mod.settings

        cs.quality = 15

        cs.mass = 0.15

        cs.tension_stiffness = 0.1
        cs.compression_stiffness = 0.05
        cs.shear_stiffness = 0.01
        cs.bending_stiffness = 1 * k

        cs.tension_damping = 0.5 * k
        cs.compression_damping = 0.5 * k
        cs.shear_damping = 0.5 * k
        cs.bending_damping = 0.5

        cs.use_internal_springs = True
        cs.internal_tension_stiffness = 0.001
        cs.internal_compression_stiffness = 0.001

        cs.use_pressure = True
        cs.pressure_factor = 400 * k * k
        cs.fluid_density = 20 * k * k * k

        cs.shrink_min = 0.05
        cs.vertex_group_mass = "pin"

        cs.vertex_group_structural_stiffness = "stiffness"
        cs.tension_stiffness_max = 0.15
        cs.compression_stiffness_max = 0.15
        cs.vertex_group_shear_stiffness = "stiffness"
        cs.shear_stiffness_max = 0.2
        cs.vertex_group_bending = "stiffness"
        cs.bending_stiffness_max = 24 * k

        return cs

    def make_pin_vgroup(self, obj):
        vg = obj.vertex_groups.new(name="pin")

        vg.add(list(range(self.base_start_idx, self.vertex_count)), 1.0, 'REPLACE')

    def make_stiffness_vgroup(self, obj):
        vg = obj.vertex_groups.new(name="stiffness")

        vg.add(list(range(0, 1 + 3*self.row_size)), 0.5, 'REPLACE')
        vg.add(list(range(0, 1 + 2*self.row_size)), 0.75, 'REPLACE')
        vg.add(list(range(0, 1 + self.row_size)), 1.0, 'REPLACE')

    def make_bottom_vgroup(self, obj, radius):
        vg = obj.vertex_groups.new(name="bottom")
        verts = obj.data.vertices

        for i in range(0, self.base_start_idx):
            co = verts[i].co
            weight = max(0, 0.5 * co[1] - 0.866 * co[2]) * 0.5 / radius
            if weight > 0:
                vg.add([i], weight, 'REPLACE')

    def make_deform_vgroups(self, obj, pbone):
        parent_name = (pbone.parent.name if pbone.parent else 'parent')
        vgp = obj.vertex_groups.new(name="DEF-" + parent_name)
        vgb = obj.vertex_groups.new(name="DEF-" + pbone.name)

        vgb.add([0], 1.0, 'REPLACE')
        vgp.add(list(range(self.base_start_idx, self.vertex_count)), 1.0, 'REPLACE')

        for i in range(0, self.row_cnt-1):
            factor = math.cos((i + 1) * math.pi / 2 / self.row_cnt)
            verts = list(range(1 + i * self.row_size, 1 + (i + 1) * self.row_size))
            vgb.add(verts, factor, 'REPLACE')
            vgp.add(verts, 1-factor, 'REPLACE')

    @staticmethod
    def add_weight_mix(obj, name, dest, src, option):
        mod = obj.modifiers.new(name=name, type='VERTEX_WEIGHT_MIX')
        mod.mix_mode = 'ADD'
        mod.mix_set = 'OR'
        mod.vertex_group_a = dest
        mod.vertex_group_b = src

        make_driver(mod, 'mask_constant', variables=[(obj, obj, option)])

    def execute(self, context):
        pbone = context.active_pose_bone
        size = pbone.bone.length
        matrix = context.object.matrix_world @ pbone.bone.matrix_local
        name = 'CAGE-' + pbone.name

        mesh = bpy.data.meshes.new(name)
        self.create_mesh_data(mesh, size, 5, 8)

        obj = bpy.data.objects.new(name, mesh)
        obj.matrix_world = matrix
        obj.display_type = 'WIRE'
        obj.hide_render = True

        # Properties
        make_property(obj, 'option_physics', 0, description='Physics simulation')
        make_property(obj, 'option_pin_front', 0.0, description='Pin front section of the mesh')
        make_property(obj, 'option_up', 0.0, description='Push up')

        # Vertex groups
        self.make_pin_vgroup(obj)
        self.make_stiffness_vgroup(obj)
        self.make_bottom_vgroup(obj, size)
        self.make_deform_vgroups(obj, pbone)

        self.add_weight_mix(obj, 'Pin Front', 'pin', 'stiffness', 'option_pin_front')
        self.add_weight_mix(obj, 'Up', 'pin', 'bottom', 'option_up')

        obj.modifiers.new(name='Armature', type='ARMATURE')

        cs = self.add_cloth_sim(obj, size)

        # Shape keys
        obj.shape_key_add(name='basis')

        rest_sk = obj.shape_key_add(name='rest')
        rest_sk.value = 0
        cs.rest_shape_key = rest_sk

        self.make_up_shape_key(obj)

        # Anchor
        anchor = bpy.data.objects.new('ANCHOR-' + pbone.name, None)
        anchor.empty_display_size = size / 2
        anchor.hide_render = True
        anchor.parent_vertices = [1, 1 + int(self.row_size / 3), 1 + int(self.row_size * 2 / 3)]
        anchor.parent = obj
        anchor.parent_type = 'VERTEX_3'

        # Finalize
        context.collection.objects.link(obj)
        context.collection.objects.link(anchor)

        parameters = pbone.rigify_parameters  # noqa
        parameters.jiggle_cloth_cage = obj
        parameters.jiggle_front_anchor = anchor
        return {'FINISHED'}


# noinspection PyPep8Naming
class MESH_OT_rigify_add_jiggle_shapekey_anchor(bpy.types.Operator):
    bl_idname = 'mesh.rigify_add_jiggle_shapekey_anchor'
    bl_label = "Add Shapekey Anchor"
    bl_description = "Add a linked duplicate of the cage and an anchor tracking shape keys"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    @classmethod
    def poll(cls, context):
        pbone = context.active_pose_bone
        if not pbone:
            return False

        parameters = pbone.rigify_parameters  # noqa
        cage = parameters.jiggle_cloth_cage
        anchor = parameters.jiggle_front_anchor
        return cage and anchor and anchor.parent == cage

    def execute(self, context):
        pbone = context.active_pose_bone
        parameters = pbone.rigify_parameters  # noqa
        cage = parameters.jiggle_cloth_cage
        anchor = parameters.jiggle_front_anchor

        cage_copy = bpy.data.objects.new(cage.name + '-SHAPE', cage.data)
        cage_copy.parent = cage.parent
        cage_copy.matrix_parent_inverse = cage.matrix_parent_inverse
        cage_copy.matrix_world = cage.matrix_world
        cage_copy.display_type = 'WIRE'
        cage_copy.hide_render = True

        anchor_copy = anchor.copy()
        anchor_copy.name = anchor.name + '-SHAPE'
        anchor_copy.parent = cage_copy
        anchor_copy.matrix_parent_inverse = anchor.matrix_parent_inverse
        anchor_copy.hide_render = True

        context.collection.objects.link(cage_copy)
        context.collection.objects.link(anchor_copy)

        parameters.jiggle_shape_anchor = anchor_copy
        return {'FINISHED'}
