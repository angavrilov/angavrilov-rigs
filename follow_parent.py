import bpy

import re
import itertools
import bisect

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


class FollowParentBuilder(GeneratorPlugin, MechanismUtilityMixin):
    """
    Implements centralized generation of switchable follow parent controls.
    Allows all rigs to register their bones as possible parents for other rigs.
    """

    def __init__(self, generator):
        super(FollowParentBuilder,self).__init__(generator)

        self.child_list = []
        self.global_parents = []
        self.local_parents = []
        self.frozen = False

        self.register_parent(None, 'root', name='Root', is_global=True)


    ##############################
    # API

    def register_parent(self, rig, bone, *, name=None, is_global=False, exclude_self=False):
        """
        Registers a bone of the specified rig as a possible parent.

        Parameters:
          rig               Owner of the bone.
          bone              Actual name of the parent bone, or a function returning it.
          name              Name of the parent for mouse-over hint.
          is_global         The parent is accessible to all rigs, instead of just children of owner.
          exclude_self      The parent is invisible to the owner rig itself.
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


    def build_child(self, rig, bone, *, prop_bone=None, prop_id=None, prop_name=None, controls=None, extra_parents=[], use_parent_mch=True):
        """
        Build a switchable follow parent mechanism for the specified bone.

        Parameters:
          rig               Owner of the child bone.
          bone              Name of the child bone.
          prop_bone         Name of the bone to add the property to, or function returning it.
          prop_id           Actual name of the control property.
          prop_name         Name of the property to use in the UI script.
          controls          Collection of controls to bind property UI to, or a function returning it.
          extra_parents     List of bone names or (name, user_name) pairs to use as additional parents.
          use_parent_mch    Create an intermediate MCH bone for the constraints and parent the child to it.
        """
        assert self.generator.stage == 'generate_bones' and not self.frozen
        assert rig is not None
        assert isinstance(bone, str)

        # Create MCH proxy
        if use_parent_mch:
            mch_bone = rig.copy_bone(bone, make_derived_name(bone, 'mch', '.parent'))
            rig.get_bone(mch_bone).length /= 3
        else:
            mch_bone = bone

        self.child_list.append({
            'rig':rig, 'bone': bone, 'mch_bone': mch_bone,
            'prop_bone': prop_bone, 'prop_id': prop_name, 'prop_name': prop_name,
            'controls': controls, 'extra_parents': extra_parents,
        })

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
                if parent['exclude_self'] and parent['rig'] is child_rig:
                    continue

                if parent['is_global'] or _rig_is_child(child_rig, parent['rig']):
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
        script = self.generator.script

        for child in self.child_list:
            bone = child['bone']

            # Build the final list of parent bone names
            parent_bones = [ (parent['bone'], parent['name']) for parent in child['parents'] ]
            parent_bones += [
                parent if isinstance(parent, tuple) else (parent, parent)
                for parent in child['extra_parents']
            ]

            child['parent_bones'] = parent_bones

            # Create the controlling property
            prop_bone = child['prop_bone'] = _auto_call(child['prop_bone']) or bone
            prop_name = child['prop_name'] or child['prop_id'] or 'Parent Switch'
            prop_id = child['prop_id'] = child['prop_id'] or 'parent_switch'

            parent_names = [ 'None' ] + [ parent[1] or strip_prefix(parent[0]) for parent in parent_bones ]

            self.make_property(
                prop_bone, prop_id, len(parent_bones),
                min=0, max=len(parent_bones),
                description='Switch parent: ' + ', '.join(parent_names)
            )

            # Create the script for the property
            controls = _auto_call(child['controls']) or set([prop_bone, bone])

            script.add_panel_selected_check(controls)
            script.add_panel_custom_prop(prop_bone, prop_id, text=prop_name)

    def rig_bones(self):
        for child in self.child_list:
            # Implement via an Armature constraint
            con = self.make_constraint(child['mch_bone'], 'ARMATURE')

            prop_var = [(child['prop_bone'], child['prop_id'])]

            for i, (parent, parent_name) in enumerate(child['parent_bones']):
                tgt = con.targets.new()

                tgt.target = self.obj
                tgt.subtarget = parent
                tgt.weight = 0.0

                expr = 'var == %d' % (i+1)
                self.make_driver(tgt, 'weight', expression=expr, variables=prop_var)
