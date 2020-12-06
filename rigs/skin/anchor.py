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

from .skin_rigs import BaseSkinChainRigWithRotationOption, ControlBoneNode, ControlNodeIcon

from rigify.rigs.basic.raw_copy import RelinkConstraintsMixin


class Rig(BaseSkinChainRigWithRotationOption, RelinkConstraintsMixin):
    """Custom skin control node."""

    chain_priority = 20

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.make_deform = self.params.make_extra_deform


    ####################################################
    # CONTROL NODES

    @stage.initialize
    def init_control_nodes(self):
        org = self.bones.org
        name = make_derived_name(org, 'ctrl')

        self.control_node = node = ControlBoneNode(self, org, name, icon=ControlNodeIcon.FREE)

        node.hide_control = self.params.skin_anchor_hide

    def extend_control_node_rig(self, node):
        if node.rig == self:
            self.copy_bone_properties(self.bones.org, node.control_bone)

            self.relink_bone_constraints(self.bones.org)

            # Move constraints to the control
            org_bone = self.get_bone(self.bones.org)
            ctl_bone = self.get_bone(node.control_bone)

            for con in list(org_bone.constraints):
                ctl_bone.constraints.copy(con)
                org_bone.constraints.remove(con)


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
        params.make_extra_deform = bpy.props.BoolProperty(
            name        = "Extra Deform",
            default     = False,
            description = "Create an optional deform bone"
        )

        params.skin_anchor_hide = bpy.props.BoolProperty(
            name        = 'Suppress Control',
            default     = False,
            description = 'Make the control bone a mechanism bone invisible to the user and only affected by constraints'
        )

        self.add_relink_constraints_params(params)

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "make_extra_deform", text='Generate Deform Bone')
        layout.prop(params, "skin_anchor_hide")
        layout.prop(params, "relink_constraints")

        super().parameters_ui(layout, params)
