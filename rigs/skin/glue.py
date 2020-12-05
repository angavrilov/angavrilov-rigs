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

from .skin_rigs import BaseSkinRig, ControlQueryNode, PositionQueryNode

from rigify.rigs.basic.raw_copy import RelinkConstraintsMixin


class Rig(BaseSkinRig, RelinkConstraintsMixin):
    """Custom skin control query node."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.use_tail = self.params.relink_constraints and self.params.skin_glue_use_tail
        self.relink_unmarked_constraints = self.use_tail


    ####################################################
    # CONTROL NODES

    @stage.initialize
    def init_control_nodes(self):
        bone = self.get_bone(self.base_bone)

        self.head_position_node = PositionQueryNode(
            self, self.base_bone, point=bone.head, rig_org=True,
            needs_reparent=self.params.skin_glue_head_reparent,
        )

        self.head_constraint_node = ControlQueryNode(
            self, self.base_bone, point=bone.head
        )

        if self.use_tail:
            self.tail_position_node = PositionQueryNode(
                self, self.base_bone, point=bone.tail,
                needs_reparent=self.params.skin_glue_tail_reparent,
            )

    def build_control_node_parent(self, node):
        return self.build_control_node_parent_next(node)


    ##############################
    # ORG chain

    @stage.rig_bones
    def rig_org_chain(self):
        # This executes before head_position_node owned a by generator plugin
        self.relink_bone_constraints(self.bones.org)

        # Move constraints to the control
        org_bone = self.get_bone(self.bones.org)
        ctl_bone = self.get_bone(self.head_constraint_node.control_bone)

        for con in list(org_bone.constraints):
            ctl_bone.constraints.copy(con)
            org_bone.constraints.remove(con)

    def find_relink_target(self, spec, old_target):
        if self.use_tail and (spec == 'TARGET' or spec == '' == old_target):
            return self.tail_position_node.output_bone

        return super().find_relink_target(spec, old_target)


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.skin_glue_head_reparent = bpy.props.BoolProperty(
            name        = 'Localize Parent Transform',
            default     = False,
            description = 'Include transformations induced by parents into local space'
        )

        params.skin_glue_use_tail = bpy.props.BoolProperty(
            name        = 'Use Tail Target',
            default     = False,
            description = 'Relink TARGET to control at the bone tail location'
        )

        params.skin_glue_tail_reparent = bpy.props.BoolProperty(
            name        = 'Localize Target Parent',
            default     = False,
            description = 'Include transformations induced by target parents into target local space'
        )

        self.add_relink_constraints_params(params)

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "skin_glue_head_reparent")
        layout.prop(params, "relink_constraints")

        col = layout.column()
        col.active = params.relink_constraints
        col.prop(params, "skin_glue_use_tail")

        col2 = col.column()
        col2.active = params.skin_glue_use_tail
        col2.prop(params, "skin_glue_tail_reparent")

