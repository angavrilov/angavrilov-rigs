rigify_info = {
    "name": "Experimental Rigs by Alexander Gavrilov",
    "link": "https://github.com/angavrilov/angavrilov-rigs",
}

from bpy.utils import register_class, unregister_class

from .rigs.jiggle.cloth_cage import MESH_OT_rigify_add_jiggle_cloth_cage, MESH_OT_rigify_add_jiggle_shapekey_anchor
from .rigs.basic.center_of_mass import MESH_OT_rigify_add_com_volume_cage

def register():
    #print("Alexander Gavrilov rigs registered")
    register_class(MESH_OT_rigify_add_jiggle_cloth_cage)
    register_class(MESH_OT_rigify_add_jiggle_shapekey_anchor)
    register_class(MESH_OT_rigify_add_com_volume_cage)

def unregister():
    #print("Alexander Gavrilov rigs unregistered")
    unregister_class(MESH_OT_rigify_add_jiggle_cloth_cage)
    unregister_class(MESH_OT_rigify_add_jiggle_shapekey_anchor)
    unregister_class(MESH_OT_rigify_add_com_volume_cage)
