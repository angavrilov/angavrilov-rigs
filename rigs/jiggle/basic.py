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

from itertools import count

from rigify.utils.rig import connected_children_names
from rigify.utils.bones import put_bone, copy_bone_properties, align_bone_orientation, set_bone_widget_transform
from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.misc import map_list
from rigify.utils.widgets import create_widget

from rigify.base_rig import stage, BaseRig

from rigify.rigs.basic.raw_copy import RelinkConstraintsMixin


class Rig(BaseRig, RelinkConstraintsMixin):
    """
    Basic jiggle setup found in some rigs on the internet. Creates two grab
    controls with the deform bone automatically stretching between them.

    The chain should consist of one or two bones. If present, constraints on
    the ORG bones are transplanted to helper parent bones for the controls.
    """

    def find_org_bones(self, bone):
        return [bone.name] + connected_children_names(self.obj, bone.name)

    def initialize(self):
        if len(self.bones.org) not in {1, 2}:
            self.raise_error("Input to rig type must be a chain of 1 or 2 bones.")

        self.has_constraints = [
            len(self.get_bone(self.bones.org[0]).constraints) > 0,
            len(self.get_bone(self.bones.org[-1]).constraints) > 0,
        ]

    def parent_bones(self):
        self.rig_parent_bone = self.get_bone_parent(self.bones.org[0])

    def prepare_bones(self):
        org = self.bones.org

        if len(org) > 1:
            align_bone_orientation(self.obj, org[1], org[0])

    ##############################
    # UTILITIES

    def make_bone_copy(self, is_front, name, **options):
        org = self.bones.org[0]
        name = self.copy_bone(org, name, **options)
        if is_front:
            put_bone(self.obj, name, self.get_bone(org).tail)
        return name

    ##############################
    # BONES
    #
    # org[]:
    #   ORG bones
    # mch:
    #   back, front:
    #     Jiggle parent bones.
    # ctrl:
    #   back, front:
    #     Jiggle controls.
    # deform:
    #   Deform bone
    #
    ##############################

    ##############################
    # Constraint MCH chain

    @stage.generate_bones
    def make_constraint_mch_chain(self):
        org = self.bones.org[0]

        if self.has_constraints[0]:
            self.bones.mch.back = self.make_bone_copy(False, make_derived_name(org, 'mch', '.back'), scale=0.5)
        if self.has_constraints[1]:
            self.bones.mch.front = self.make_bone_copy(True, make_derived_name(org, 'mch', '.front'), scale=0.5)

    @stage.parent_bones
    def parent_constraint_mch_chain(self):
        if self.has_constraints[0]:
            self.set_bone_parent(self.bones.mch.back, self.rig_parent_bone)
        if self.has_constraints[1]:
            self.set_bone_parent(self.bones.mch.front, self.rig_parent_bone)

    @stage.configure_bones
    def setup_constraint_mch_chain(self):
        if self.has_constraints[0]:
            self.copy_constraints(self.bones.org[0], self.bones.mch.back)
            self.relink_bone_constraints(self.bones.mch.back)

        if self.has_constraints[1]:
            self.copy_constraints(self.bones.org[-1], self.bones.mch.front)
            self.relink_bone_constraints(self.bones.mch.front)

        for org in self.bones.org:
            self.clear_constraints(org)

    def copy_constraints(self, src, dest):
        dest_bone = self.get_bone(dest)
        for con in self.get_bone(src).constraints:
            dest_bone.constraints.copy(con)

    def clear_constraints(self, name):
        bone = self.get_bone(name)
        for con in list(bone.constraints):
            bone.constraints.remove(con)

    ##############################
    # Control chain

    @stage.generate_bones
    def make_control_chain(self):
        self.bones.ctrl.back = self.make_back_control_bone(self.bones.org[0], self.bones.org[-1])
        self.bones.ctrl.front = self.make_front_control_bone(self.bones.org[0], self.bones.org[-1])

    def make_back_control_bone(self, org, end_org):
        name = make_derived_name(org, 'ctrl')
        return self.make_bone_copy(False, name)

    def make_front_control_bone(self, org, end_org):
        name = make_derived_name(end_org, 'ctrl', '_front' if end_org == org else '')
        return self.make_bone_copy(True, name)

    @stage.parent_bones
    def parent_control_chain(self):
        self.set_bone_parent(self.bones.ctrl.back, self.bones.mch.back if self.has_constraints[0] else self.rig_parent_bone)
        self.set_bone_parent(self.bones.ctrl.front, self.bones.mch.front if self.has_constraints[1] else self.rig_parent_bone)

    @stage.configure_bones
    def configure_control_chain(self):
        for args in zip(count(0), [self.bones.ctrl.back, self.bones.ctrl.front]):
            self.configure_control_bone(*args)

    def configure_control_bone(self, i, ctrl):
        copy_bone_properties(self.obj, self.bones.org[-i], ctrl)

        bone = self.get_bone(ctrl)
        bone.lock_rotation = True, True, True
        bone.lock_rotation_w = True
        bone.lock_scale = True, True, True

    @stage.rig_bones
    def rig_control_chain(self):
        if self.params.jiggle_follow_front > 0:
            ctrl = self.bones.ctrl
            self.make_constraint(
                ctrl.back, 'COPY_LOCATION', ctrl.front,
                use_y=False, use_offset=True, space='LOCAL',
                influence=self.params.jiggle_follow_front
            )

    @stage.generate_widgets
    def make_control_widgets(self):
        create_back_widget(self.obj, self.bones.ctrl.back)
        set_bone_widget_transform(self.obj, self.bones.ctrl.back, self.bones.org[0])

        create_front_widget(self.obj, self.bones.ctrl.front)
        set_bone_widget_transform(self.obj, self.bones.ctrl.front, self.bones.org[0])

    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        ctrl = self.bones.ctrl
        for org, ctl in zip(self.bones.org, [ctrl.back, ctrl.front]):
            self.set_bone_parent(org, ctl, use_connect=False)

    @stage.rig_bones
    def rig_org_chain(self):
        for func, org in zip([self.rig_back_org_bone, self.rig_front_org_bone], self.bones.org):
            func(org)

    def rig_back_org_bone(self, org):
        self.make_constraint(org, 'STRETCH_TO', self.bones.ctrl.front, keep_axis='SWING_Y')

    def rig_front_org_bone(self, org):
        self.make_constraint(org, 'DAMPED_TRACK', self.bones.org[0], track_axis='-Y')

    ##############################
    # Deform chain

    @stage.generate_bones
    def make_deform_chain(self):
        org = self.bones.org[0]
        self.bones.deform = self.copy_bone(org, make_derived_name(org, 'def'))

    @stage.parent_bones
    def parent_deform_chain(self):
        self.set_bone_parent(self.bones.deform, self.bones.org[0])

    ##############################
    # UI

    @classmethod
    def add_parameters(self, params):
        self.add_relink_constraints_params(params)

        params.jiggle_follow_front = bpy.props.FloatProperty(
            name="Follow Front", default=0.0, min=0.0, max=1.0,
            description="The back control follows the front one"
            )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.row().prop(params, "relink_constraints")
        layout.row().prop(params, "jiggle_follow_front", slider=True)


def create_back_widget(rig, bone_name, size=1.0, bone_transform_name=None):
    obj = create_widget(rig, bone_name, bone_transform_name)
    if obj is not None:
        verts = [(3.63161e-07*size, -6.80926e-08*size, 0.5*size), (3.63161e-07*size, 7.94414e-08*size, -0.5*size),
                 (0.5*size, 4.53951e-08*size, -1.4186e-07*size), (-0.5*size, 4.53951e-08*size, -1.4186e-07*size),
                 (-0.5*size, 0.190058*size, -1.02139e-07*size), (0.5*size, 0.190058*size, -1.02139e-07*size),
                 (3.63161e-07*size, 0.190058*size, 0.5*size), (3.63161e-07*size, 0.190058*size, -0.5*size),
                 ]
        edges = [(1, 0), (3, 2), (4, 3), (2, 5), (0, 6), (7, 1), ]
        faces = []

        mesh = obj.data
        mesh.from_pydata(verts, edges, faces)
        mesh.update()
        return obj
    else:
        return None

def create_front_widget(rig, bone_name, size=1.0, bone_transform_name=None):
    obj = create_widget(rig, bone_name, bone_transform_name)
    if obj is not None:
        verts = [(-0.119293*size, 1.06882*size, -1.13704e-07*size), (-0.234082*size, 1.04806*size, -1.38422e-07*size),
                 (-0.339872*size, 1.00363*size, -1.28844e-07*size), (-0.432598*size, 0.932699*size, -9.39291e-08*size),
                 (-0.508696*size, 0.839974*size, -4.94364e-08*size), (-0.565242*size, 0.734184*size, -1.38422e-07*size),
                 (-0.600063*size, 0.619395*size, -3.95491e-08*size), (0*size, 1.06882*size, -0.119377*size),
                 (0*size, 1.04806*size, -0.234166*size), (0*size, 1.00363*size, -0.339956*size),
                 (0*size, 0.932699*size, -0.432682*size), (0*size, 0.839974*size, -0.50878*size),
                 (0*size, 0.734183*size, -0.565326*size), (0*size, 0.619394*size, -0.600147*size),
                 (0*size, 0.500018*size, -0.611904*size), (0.119293*size, 1.06882*size, -1.13704e-07*size),
                 (0.234082*size, 1.04806*size, -1.38422e-07*size), (0.339872*size, 1.00363*size, -1.28844e-07*size),
                 (0.432598*size, 0.932699*size, -9.39291e-08*size), (0.508696*size, 0.839974*size, -4.94364e-08*size),
                 (0.565242*size, 0.734184*size, -1.38422e-07*size), (0.600063*size, 0.619395*size, -3.95491e-08*size),
                 (0.611821*size, 0.500018*size, -3.95491e-08*size), (0*size, 1.06882*size, 0.119377*size),
                 (0*size, 1.04806*size, 0.234165*size), (0*size, 1.00363*size, 0.339956*size),
                 (0*size, 0.9327*size, 0.432681*size), (0*size, 0.839974*size, 0.508779*size),
                 (0*size, 0.734184*size, 0.565326*size), (0*size, 0.619395*size, 0.600146*size),
                 (0*size, 0.500018*size, 0.611904*size), (0*size, 1.06916*size, -4.44927e-08*size),
                 (-0.611821*size, 0.500018*size, -3.95491e-08*size), ]
        edges = [(13, 14), (12, 13), (11, 12), (10, 11), (9, 10), (8, 9), (7, 8), (5, 6), (4, 5), (3, 4),
                 (2, 3), (1, 2), (0, 1), (21, 22), (20, 21), (19, 20), (18, 19), (17, 18), (16, 17), (15, 16),
                 (29, 30), (28, 29), (27, 28), (26, 27), (25, 26), (24, 25), (23, 24), (31, 7), (31, 0), (31, 15),
                 (31, 23), (6, 32), ]
        faces = []

        mesh = obj.data
        mesh.from_pydata(verts, edges, faces)
        mesh.update()
        return obj
    else:
        return None


def create_sample(obj):
    """ Create a sample metarig for this rig type.
    """
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('Bone')
    bone.head[:] = 0.0000, 0.0000, 0.0000
    bone.tail[:] = 0.0000, 0.0000, 0.2000
    bone.roll = 0.0000
    bone.use_connect = False
    bones['Bone'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['Bone']]
    pbone.rigify_type = 'jiggle.basic'
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
