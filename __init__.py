rigify_info = {
    "name": "Experimental Rigs by Alexander Gavrilov",
    "link": "https://github.com/angavrilov/angavrilov-rigs",
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
    from .rigs.jiggle import cloth_fit_basis

    for cls in _get_classes():
        register_class(cls)

    cloth_fit_basis.register()


def unregister():
    from bpy.utils import unregister_class
    from .rigs.jiggle import cloth_fit_basis

    for cls in _get_classes():
        unregister_class(cls)

    cloth_fit_basis.unregister()
