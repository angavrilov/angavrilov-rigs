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

from .skin_rigs import BaseSkinRig, ControlQueryNode

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

        head_mode = self.params.skin_glue_head_mode

        self.head_position_node = PositionQueryNode(
            self, self.base_bone, point=bone.head,
            rig_org = (head_mode != 'CHILD'),
            needs_reparent = (head_mode == 'REPARENT'),
        )

        self.head_constraint_node = ControlQueryNode(
            self, self.base_bone, point=bone.head
        )

        if self.use_tail:
            self.tail_position_node = PositionQueryNode(
                self, self.base_bone, point=bone.tail,
                needs_reparent=self.params.skin_glue_tail_reparent,
            )

    def build_own_control_node_parent(self, node):
        return self.build_control_node_parent_next(node)


    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_bone(self):
        if self.params.skin_glue_head_mode == 'CHILD':
            self.set_bone_parent(self.bones.org, self.head_position_node.output_bone)

    @stage.rig_bones
    def rig_org_bone(self):
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
        params.skin_glue_head_mode = bpy.props.EnumProperty(
            name        = 'Glue Mode',
            items       = [('CHILD', 'Child Of Control',
                            "The glue bone becomes a child of the control bone"),
                           ('MIRROR', 'Mirror Of Control',
                            "The glue bone becomes a sibling of the control bone with Copy Transforms"),
                           ('REPARENT', 'Mirror With Parents',
                            "The glue bone keeps its parent, but uses Copy Transforms to group both local and parent induced motion of the control into local space")],
            default     = 'CHILD',
            description = "Specifies how the glue bone is rigged to the control at the bone head location",
        )

        params.skin_glue_use_tail = bpy.props.BoolProperty(
            name        = 'Use Tail Target',
            default     = False,
            description = 'Find the control at the bone tail location and use it to relink TARGET or any constraints without an assigned subtarget or relink spec'
        )

        params.skin_glue_tail_reparent = bpy.props.BoolProperty(
            name        = 'Target Local With Parents',
            default     = False,
            description = 'Include transformations induced by target parents into target local space'
        )

        self.add_relink_constraints_params(params)

        super().add_parameters(params)

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "skin_glue_head_mode")
        layout.prop(params, "relink_constraints")

        col = layout.column()
        col.active = params.relink_constraints
        col.prop(params, "skin_glue_use_tail")

        col2 = col.column()
        col2.active = params.skin_glue_use_tail
        col2.prop(params, "skin_glue_tail_reparent")


class PositionQueryNode(ControlQueryNode):
    """Finds the position of the highest layer control and rig reparent and/or org bone"""

    def __init__(self, rig, org, *, point=None, needs_reparent=False, rig_org=False):
        super().__init__(rig, org, point=point, find_highest_layer=True)

        self.needs_reparent = needs_reparent
        self.rig_org = rig_org

    @property
    def output_bone(self):
        if self.rig_org:
            return self.org
        elif self.needs_reparent:
            return self.merged_master.get_reparent_bone(self.node_parent)
        else:
            return self.control_bone

    def initialize(self):
        if self.needs_reparent:
            self.node_parent = self.merged_master.build_parent_for_node(self)

            if not self.rig_org:
                self.merged_master.request_reparent(self.node_parent)

    def parent_bones(self):
        if self.rig_org:
            if self.needs_reparent:
                parent = self.node_parent.output_bone
            else:
                parent = self.get_bone_parent(self.control_bone)

            self.set_bone_parent(self.org, parent, inherit_scale='AVERAGE')

    def apply_bones(self):
        if self.rig_org:
            self.get_bone(self.org).matrix = self.merged_master.matrix

    def rig_bones(self):
        if self.rig_org:
            self.make_constraint(self.org, 'COPY_TRANSFORMS', self.control_bone)
