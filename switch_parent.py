import bpy

import re
import itertools
import bisect
import json

from rigify.utils.errors import MetarigError
from rigify.utils.naming import strip_prefix, make_derived_name
from rigify.utils.mechanism import MechanismUtilityMixin
from rigify.utils.misc import map_list, map_apply

from rigify.base_rig import *
from rigify.base_generate import GeneratorPlugin

from itertools import count, repeat


def _auto_call(value):
    if callable(value):
        return value()
    else:
        return value

def _rig_is_child(rig, parent):
    if parent is None:
        return True

    while rig:
        if rig is parent:
            return True

        rig = rig.rigify_parent

    return False


class SwitchParentBuilder(GeneratorPlugin, MechanismUtilityMixin):
    """
    Implements centralized generation of switchable parent mechanisms.
    Allows all rigs to register their bones as possible parents for other rigs.
    """

    def __init__(self, generator):
        super(SwitchParentBuilder,self).__init__(generator)

        self.child_list = []
        self.global_parents = []
        self.local_parents = []
        self.child_map = {}
        self.frozen = False

        self.register_parent(None, 'root', name='Root', is_global=True)


    ##############################
    # API

    def register_parent(self, rig, bone, *, name=None, is_global=False, exclude_self=False):
        """
        Registers a bone of the specified rig as a possible parent.

        Parameters:
          rig               Owner of the bone.
          bone              Actual name of the parent bone.
          name              Name of the parent for mouse-over hint.
          is_global         The parent is accessible to all rigs, instead of just children of owner.
          exclude_self      The parent is invisible to the owner rig itself.

        Lazy creation:
          The bone parameter may be a function creating the bone on demand and
          returning its name. It is guaranteed to be called at most once.
        """

        assert not self.frozen
        assert isinstance(bone, str) or callable(bone)

        entry = {
            'rig': rig, 'bone': bone, 'name': name,
            'is_global': is_global, 'exclude_self': exclude_self, 'used': False,
        }

        if is_global:
            self.global_parents.append(entry)
        else:
            self.local_parents.append(entry)


    def build_child(self, rig, bone, *, extra_parents=[], use_parent_mch=True,
                    prop_bone=None, prop_id=None, prop_name=None, controls=None,
                    ctrl_bone=None,
                    no_fix_location=False, no_fix_rotation=False, no_fix_scale=False,
                    copy_location=None, copy_rotation=None, copy_scale=None):
        """
        Build a switchable parent mechanism for the specified bone.

        Parameters:
          rig               Owner of the child bone.
          bone              Name of the child bone.
          extra_parents     List of bone names or (name, user_name) pairs to use as additional parents.
          use_parent_mch    Create an intermediate MCH bone for the constraints and parent the child to it.

          prop_bone         Name of the bone to add the property to.
          prop_id           Actual name of the control property.
          prop_name         Name of the property to use in the UI script.
          controls          Collection of controls to bind property UI to.

          ctrl_bone         User visible control bone that depends on this parent (for switch & keep transform)
          no_fix_*          Disable "Switch and Keep Transform" correction for specific channels.
          copy_*            Override the specified components by copying from another bone.

        Lazy parameters:
          'extra_parents', 'prop_bone', 'controls', 'copy_*' may be a function
          returning the value. They are called in the configure_bones stage.
        """
        assert self.generator.stage == 'generate_bones' and not self.frozen
        assert rig is not None
        assert isinstance(bone, str)
        assert bone not in self.child_map

        # Create MCH proxy
        if use_parent_mch:
            mch_bone = rig.copy_bone(bone, make_derived_name(bone, 'mch', '.parent'))
            rig.get_bone(mch_bone).length /= 3
        else:
            mch_bone = bone

        child = {
            'rig':rig, 'bone': bone, 'mch_bone': mch_bone,
            'prop_bone': prop_bone, 'prop_id': prop_name, 'prop_name': prop_name,
            'controls': controls, 'extra_parents': extra_parents or [],
            'ctrl_bone': ctrl_bone,
            'no_fix': (no_fix_location, no_fix_rotation, no_fix_scale),
            'copy': (copy_location, copy_rotation, copy_scale),
            'is_done': False
        }
        self.child_list.append(child)
        self.child_map[bone] = child


    def rig_child_now(self, bone):
        """Create the constraints immediately."""
        assert self.generator.stage == 'rig_bones'
        child = self.child_map[bone]
        assert not child['is_done']
        self.__rig_child(child)

    ##############################
    # Implementation

    def generate_bones(self):
        self.frozen = True
        self.parent_list = self.global_parents + self.local_parents

        # Link children to parents
        for child in self.child_list:
            child_rig = child['rig']
            parents = []

            for parent in self.parent_list:
                if parent['rig'] is child_rig:
                    if parent['exclude_self']:
                        continue
                elif parent['is_global']:
                    # Can't use parents from own children, even if global (cycle risk)
                    if _rig_is_child(parent['rig'], child_rig):
                        continue
                else:
                    # Required to be a child of the rig
                    if _rig_is_child(child_rig, parent['rig']):
                        continue

                parent['used'] = True
                parents.append(parent)

            child['parents'] = parents

        # Call lazy creation for parents
        for parent in self.parent_list:
            if parent['used']:
                parent['bone'] = _auto_call(parent['bone'])

    def parent_bones(self):
        for child in self.child_list:
            rig = child['rig']
            mch = child['mch_bone']

            # Remove real parent from the child
            rig.set_bone_parent(mch, None)
            self.generator.disable_auto_parent(mch)

            # Parent child to the MCH proxy
            if mch != child['bone']:
                rig.set_bone_parent(child['bone'], mch)

    def configure_bones(self):
        for child in self.child_list:
            self.__configure_child(child)

    def __configure_child(self, child):
        bone = child['bone']

        # Build the final list of parent bone names
        parent_map = dict()

        for parent in child['parents']:
            if parent['bone'] not in parent_map:
                parent_map[parent['bone']] = parent['name']

        for parent in _auto_call(child['extra_parents']):
            if not isinstance(parent, tuple):
                parent = (parent, None)
            if parent[0] not in parent_map:
                parent_map[parent[0]] = parent[1]

        parent_bones = list(parent_map.items())
        child['parent_bones'] = parent_bones

        # Create the controlling property
        prop_bone = child['prop_bone'] = _auto_call(child['prop_bone']) or bone
        prop_name = child['prop_name'] or child['prop_id'] or 'Parent Switch'
        prop_id = child['prop_id'] = child['prop_id'] or 'parent_switch'

        parent_names = [ parent[1] or strip_prefix(parent[0]) for parent in [(None, 'None'), *parent_bones] ]
        parent_str = ', '.join([ '%s (%d)' % (name, i) for i, name in enumerate(parent_names) ])

        ctrl_bone = child['ctrl_bone'] or bone

        self.make_property(
            prop_bone, prop_id, len(parent_bones),
            min=0, max=len(parent_bones),
            description='Switch parent of %s: %s' % (ctrl_bone, parent_str)
        )

        # Find which channels don't depend on the parent

        child['copy'] = tuple(map(_auto_call, child['copy']))

        locks = tuple(bool(nofix or copy) for nofix, copy in zip(child['no_fix'], child['copy']))

        # Create the script for the property
        controls = _auto_call(child['controls']) or set([prop_bone, bone])

        script = self.generator.script
        panel = self.generator.script.panel_with_selected_check(controls)

        script.add_utilities(UTILITIES_FUNC_SET_TRF_WITH_LOCKS + UTILITIES_OP_SWITCH_PARENT)
        script.register_classes(REGISTER_OP_SWITCH_PARENT)

        op_name = 'pose.rigify_switch_parent_' + self.generator.rig_id
        op_props = {
            'bone': ctrl_bone, 'prop_bone': prop_bone, 'prop_id': prop_id,
            'parent_names': json.dumps(parent_names), 'locks': locks,
        }

        row = panel.row(align=True)
        row.custom_prop(prop_bone, prop_id, text=prop_name)
        row.operator(op_name, text='', icon='MODIFIER', properties=op_props)

    def rig_bones(self):
        for child in self.child_list:
            self.__rig_child(child)

    def __rig_child(self, child):
        if child['is_done']:
            return

        child['is_done'] = True

        # Implement via an Armature constraint
        mch = child['mch_bone']
        con = self.make_constraint(mch, 'ARMATURE', name='SWITCH_PARENT')

        prop_var = [(child['prop_bone'], child['prop_id'])]

        for i, (parent, parent_name) in enumerate(child['parent_bones']):
            tgt = con.targets.new()

            tgt.target = self.obj
            tgt.subtarget = parent
            tgt.weight = 0.0

            expr = 'var == %d' % (i+1)
            self.make_driver(tgt, 'weight', expression=expr, variables=prop_var)

        # Add copy constraints
        copy = child['copy']

        if copy[0]:
            self.make_constraint(mch, 'COPY_LOCATION', copy[0])
        if copy[1]:
            self.make_constraint(mch, 'COPY_ROTATION', copy[1])
        if copy[2]:
            self.make_constraint(mch, 'COPY_SCALE', copy[2])


REGISTER_OP_SWITCH_PARENT = ['Rigify_Switch_Parent']

UTILITIES_FUNC_SET_TRF_WITH_LOCKS = ['''
####################################
## Set bone transform from matrix ##
####################################

def set_transform_from_matrix_with_locks(bone, matrix, extra_locks=[False,False,False]):
    old_loc = Vector(bone.location)
    old_rot_euler = Vector(bone.rotation_euler)
    old_rot_quat = Vector(bone.rotation_quaternion)
    old_scale = Vector(bone.scale)

    bone.matrix_basis = matrix

    # Restore locked properties
    for i, val in enumerate(old_loc):
        if extra_locks[0] or bone.lock_location[i]:
            bone.location[i] = val

    for i, val in enumerate(old_rot_euler):
        if extra_locks[1] or bone.lock_rotation[i]:
            bone.rotation_euler[i] = val

    quat_locks = [bone.lock_rotation_w, *bone.lock_rotation]
    for i, (val, lock) in enumerate(zip(old_rot_quat, quat_locks)):
        if extra_locks[1] or lock:
            bone.rotation_quaternion[i] = val

    for i, val in enumerate(old_scale):
        if extra_locks[2] or bone.lock_scale[i]:
            bone.scale[i] = val
''']

UTILITIES_OP_SWITCH_PARENT = ['''
################################
## Switchable Parent operator ##
################################

class Rigify_Switch_Parent(bpy.types.Operator):
    bl_idname = "pose.rigify_switch_parent_" + rig_id
    bl_label = "Switch Parent (Keep Transform)"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Switch parent, preserving the bone position and orientation"

    bone:         StringProperty(name="Control Bone")
    prop_bone:    StringProperty(name="Property Bone")
    prop_id:      StringProperty(name="Property")
    parent_names: StringProperty(name="Parent Names")
    locks:        bpy.props.BoolVectorProperty(name="Locked", size=3, default=[False,False,False])

    parent_items = [('0','None','None')]

    selected: bpy.props.EnumProperty(
        name='Selected Parent',
        items=lambda s,c: Rigify_Switch_Parent.parent_items
    )

    def execute(self, context):
        from rna_prop_ui import rna_idprop_ui_prop_update

        obj = context.active_object
        ctrl_bone = obj.pose.bones[self.bone]
        prop_bone = obj.pose.bones[self.prop_bone]
        old_val = prop_bone[self.prop_id]
        new_val = int(self.selected)

        if old_val == new_val:
            return {'CANCELLED'}

        old_matrix = obj.convert_space(
            pose_bone=ctrl_bone, matrix=ctrl_bone.matrix_basis,
            from_space='LOCAL', to_space='WORLD'
        )

        # Change the parent
        prop_bone[self.prop_id] = new_val

        rna_idprop_ui_prop_update(prop_bone, self.prop_id)
        context.view_layer.update()

        # Set the transforms to restore position
        new_matrix = obj.convert_space(
            pose_bone=ctrl_bone, matrix=old_matrix,
            from_space='WORLD', to_space='LOCAL'
        )

        set_transform_from_matrix_with_locks(
            ctrl_bone, new_matrix, extra_locks=list(self.locks)
        )

        return {'FINISHED'}

    def invoke(self, context, _event):
        wm = context.window_manager
        pose = context.active_object.pose

        if (not pose or not self.parent_names
            or self.bone not in pose.bones
            or self.prop_bone not in pose.bones
            or self.prop_id not in pose.bones[self.prop_bone]):
            self.report({'ERROR'}, "Invalid parameters")
            return {'CANCELLED'}

        parents = json.loads(self.parent_names)
        pitems = [(str(i), name, name) for i, name in enumerate(parents)]

        Rigify_Switch_Parent.parent_items = pitems

        self.selected = str(pose.bones[self.prop_bone][self.prop_id])

        return wm.invoke_props_dialog(self)

    def draw(self, _context):
        layout = self.layout
        col = layout.column()
        col.prop(self, 'selected', expand=True)
''']
