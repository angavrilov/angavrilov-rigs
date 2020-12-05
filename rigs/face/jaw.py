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
import math

from rigify.utils.naming import make_derived_name, SideZ, get_name_side_z
from rigify.utils.bones import align_bone_z_axis, put_bone
from rigify.utils.misc import map_list, matrix_from_axis_pair

from rigify.rigs.widgets import create_jaw_widget

from rigify.base_rig import stage, RigComponent

from ..skin.skin_rigs import BaseSkinRig, ControlBoneNode, ControlBoneParentOrg, LazyRef
from ..skin.basic_chain import Rig as BasicChainRig

import mathutils

from itertools import count
from mathutils import Vector, Matrix


class Rig(BaseSkinRig):
    """Jaw rig."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()


    ####################################################
    # Control nodes

    def get_child_chain_parent(self, rig, parent_bone):
        if parent_bone == self.base_bone:
            side = get_name_side_z(rig.base_bone)
            if side == SideZ.TOP:
                return self.bones.mch.top
            if side == SideZ.BOTTOM:
                return self.bones.mch.bottom

        return parent_bone

    def build_control_node_parent(self, node, parent_bone):
        if parent_bone == self.base_bone:
            side = node.name_split[2]
            if side == SideZ.TOP:
                return ControlBoneParentOrg(LazyRef(self.bones.mch, 'top'))
            if side == SideZ.BOTTOM:
                return ControlBoneParentOrg(LazyRef(self.bones.mch, 'bottom'))

        return ControlBoneParentOrg(parent_bone)


    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control(self):
        org = self.bones.org
        name = self.copy_bone(org, make_derived_name(org, 'ctrl'), parent=True)
        self.bones.ctrl.master = name

    @stage.configure_bones
    def configure_master_control(self):
        self.copy_bone_properties(self.bones.org, self.bones.ctrl.master)

    @stage.generate_widgets
    def make_master_control_widget(self):
        ctrl = self.bones.ctrl.master
        create_jaw_widget(self.obj, ctrl)


    ####################################################
    # Tracking MCH

    @stage.generate_bones
    def make_mch_lock_bones(self):
        org = self.bones.org
        mch = self.bones.mch

        mch.lock = self.copy_bone(org, make_derived_name(org, 'mch', '_lock'), scale=1/2, parent=True)
        mch.top = self.copy_bone(org, make_derived_name(org, 'mch', '_top'), scale=1/4, parent=True)
        mch.bottom = self.copy_bone(org, make_derived_name(org, 'mch', '_bottom'), scale=1/3, parent=True)

    @stage.configure_bones
    def configure_mch_lock_bones(self):
        ctrl = self.bones.ctrl

        panel = self.script.panel_with_selected_check(self, [ctrl.master])

        self.make_property(ctrl.master, 'mouth_lock', 0.0, description='Mouth is locked closed')
        panel.custom_prop(ctrl.master, 'mouth_lock', text='Mouth Lock', slider=True)

    @stage.rig_bones
    def rig_mch_track_bones(self):
        mch = self.bones.mch
        ctrl = self.bones.ctrl

        self.make_constraint(mch.lock, 'COPY_TRANSFORMS', ctrl.master, influence=0.2)

        con = self.make_constraint(mch.top, 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])

        self.make_constraint(mch.bottom, 'COPY_TRANSFORMS', ctrl.master, influence=0.5)

        con = self.make_constraint(mch.bottom, 'COPY_TRANSFORMS', mch.lock)
        self.make_driver(con, 'influence', variables=[(ctrl.master, 'mouth_lock')])


    ####################################################
    # ORG bone

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org, self.bones.ctrl.master, inherit_scale='FULL')


    ####################################################
    # Deform bones

    @stage.generate_bones
    def make_deform_bone(self):
        org = self.bones.org
        deform = self.bones.deform
        self.bones.deform.master = self.copy_bone(org, make_derived_name(org, 'def'))

    @stage.parent_bones
    def parent_deform_chain(self):
        deform = self.bones.deform
        self.set_bone_parent(deform.master, self.bones.org)


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.make_deform = bpy.props.BoolProperty(
            name        = "Deform",
            default     = True,
            description = "Create a deform bone for the copy"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        pass
        #r = layout.row()
        #r.prop(params, "make_deform", text="Eyball And Iris Deforms")
