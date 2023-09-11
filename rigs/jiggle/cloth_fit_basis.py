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
import operator

from bpy.types import Context, Key, ShapeKey


# noinspection PyPep8Naming
class MESH_OT_fit_cloth_rest_to_basis(bpy.types.Operator):
    bl_idname = 'mesh.fit_cloth_rest_to_basis'
    bl_label = "Fit Rest Shape To Basis"
    bl_description = "Apply the inverse of the change from basis shape to the current shape " \
                     "to the rest shape key"
    bl_options = {"REGISTER", "UNDO", "INTERNAL"}

    factor: bpy.props.FloatProperty(
        "Factor", default=1.0, min=0.0, max=1.0, subtype="FACTOR",
        description="To what extent to apply the correction")

    @staticmethod
    def get_shapes(context: Context)\
            -> tuple[Key | None, ShapeKey | None, ShapeKey | None, int | None]:
        ob = context.object

        if ob and ob.type == 'MESH':
            assert isinstance(ob.data, bpy.types.Mesh)

            key = ob.data.shape_keys

            if key and key.use_relative:
                rest_shape = context.cloth.settings.rest_shape_key

                for i, shape in enumerate(key.key_blocks):
                    if shape == rest_shape and i > 0:
                        return key, key.key_blocks[0], rest_shape, i

        return None, None, None, None

    @classmethod
    def poll(cls, context):
        _key, _basis, rest, _idx = cls.get_shapes(context)

        return bool(rest)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        self.layout.prop(self, "factor")

    @staticmethod
    def get_coordinates(collection):
        data = [0] * len(collection) * 3
        collection.foreach_get('co', data)
        return data

    def execute(self, context):
        ob = context.object
        key, basis, rest, rest_index = self.get_shapes(context)

        if not rest:
            self.report({'ERROR'}, "Could not find shape keys")
            return {'CANCELLED'}

        depsgraph = context.view_layer.depsgraph
        factor = self.factor

        sos = ob.show_only_shape_key
        asi = ob.active_shape_key_index
        restore_sk = False

        try:
            # Extract the current evaluated coordinates
            eval_object = ob.evaluated_get(depsgraph)
            assert isinstance(eval_object.data, bpy.types.Mesh)

            eval_coords = self.get_coordinates(eval_object.data.vertices)

            # Pin the rest shape key
            if not sos or asi != rest_index:
                restore_sk = True
                ob.show_only_shape_key = True
                ob.active_shape_key_index = rest_index

                context.view_layer.update()

            # Prepare crazy space correction
            ob.crazyspace_eval(depsgraph, context.scene)

            # Offsets from current position to basis
            basis_coords = self.get_coordinates(basis.data)
            offsets = list(map(operator.sub, basis_coords, eval_coords))

            # Offsets converted into rest key space
            rest_offsets = []

            for i in range(0, len(offsets), 3):
                offset = ob.crazyspace_displacement_to_original(
                    vertex_index=i // 3, displacement=offsets[i: i + 3])
                rest_offsets.extend(offset * factor)

            # Apply offsets
            rest_coords = self.get_coordinates(rest.data)
            result_coords = list(map(operator.add, rest_coords, rest_offsets))

            rest.data.foreach_set('co', result_coords)

        finally:
            if restore_sk:
                ob.show_only_shape_key = sos
                ob.active_shape_key_index = asi

            ob.crazyspace_eval_clear()

        return {'FINISHED'}


def draw_panel(self: bpy.types.Panel, context: Context):
    ob = context.object
    assert isinstance(ob.data, bpy.types.Mesh)

    if ob.data.shape_keys:
        self.layout.operator(MESH_OT_fit_cloth_rest_to_basis.bl_idname)


def register():
    bpy.utils.register_class(MESH_OT_fit_cloth_rest_to_basis)
    bpy.types.PHYSICS_PT_cloth_shape.append(draw_panel)


def unregister():
    bpy.utils.unregister_class(MESH_OT_fit_cloth_rest_to_basis)
    bpy.types.PHYSICS_PT_cloth_shape.remove(draw_panel)
