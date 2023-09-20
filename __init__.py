# noinspection SpellCheckingInspection
rigify_info = {
    "name": "Experimental Rigs by Alexander Gavrilov",
    "author": "Alexander Gavrilov",
    "description":
        "Experimental and/or niche rigs made by a Rigify maintainer.\n"
        "Includes a BlenRig-like spine, Body IK (knee & elbow IK), jiggles, skin transforms, etc.",
    "link": "https://github.com/angavrilov/angavrilov-rigs",
    "blender": (4, 0, 0),
}


def _get_classes():
    from .rigs.jiggle import cloth_cage
    from .rigs.basic import center_of_mass

    return [
        cloth_cage.MESH_OT_rigify_add_jiggle_cloth_cage,
        cloth_cage.MESH_OT_rigify_add_jiggle_shapekey_anchor,
        cloth_cage.MESH_OT_rigify_mirror_jiggle_cloth_cage,
        center_of_mass.MESH_OT_rigify_add_com_volume_cage,
    ]


def register():
    from bpy.utils import register_class

    for cls in _get_classes():
        register_class(cls)


def unregister():
    from bpy.utils import unregister_class

    for cls in _get_classes():
        unregister_class(cls)
