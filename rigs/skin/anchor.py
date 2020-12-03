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

from rigify.utils.naming import make_derived_name
from rigify.utils.widgets_basic import create_cube_widget

from rigify.base_rig import stage

from .skin_rigs import BaseSkinChainRigWithRotationOption, ControlBoneNode


class Rig(BaseSkinChainRigWithRotationOption):
    """Custom skin control node."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.make_deform = self.params.make_deform

    def get_control_node_rotation(self):
        return self.get_bone(self.base_bone).bone.matrix_local.to_quaternion()


    ####################################################
    # CONTROL NODES

    @stage.initialize
    def init_control_nodes(self):
        org = self.bones.org
        name = make_derived_name(org, 'ctrl')

        self.control_node = node = ControlBoneNode(self, org, name, can_merge=False)

        node.hide_lone_control = self.params.skin_anchor_hide

    def make_control_node_widget(self, node):
        create_cube_widget(self.obj, node.control_bone)


    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org, self.control_node.control_bone)


    ##############################
    # Deform bone

    @stage.generate_bones
    def make_deform_bone(self):
        if self.make_deform:
            self.bones.deform = self.copy_bone(self.bones.org, make_derived_name(self.bones.org, 'def'))

    @stage.parent_bones
    def parent_deform_chain(self):
        if self.make_deform:
            self.set_bone_parent(self.bones.deform, self.bones.org)


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.make_deform = bpy.props.BoolProperty(
            name        = "Deform",
            default     = True,
            description = "Create a deform bone for the copy"
        )

        params.skin_anchor_hide = bpy.props.BoolProperty(
            name        = 'Hide Unless Merged',
            default     = False,
            description = 'Hide the control unless merged with another one'
        )

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "make_deform")
        layout.prop(params, "skin_anchor_hide")

        super().parameters_ui(layout, params)
