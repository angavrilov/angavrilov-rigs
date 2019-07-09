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

from rigify.utils.bones import BoneDict, compute_chain_x_axis, align_bone_x_axis
from rigify.utils.naming import make_derived_name
from rigify.utils.misc import map_list

from rigify.base_rig import stage

from .limb_rigs import BaseLimbRig


IMPLEMENTATION = True


class Rig(BaseLimbRig):
    """Human arm rig."""

    def initialize(self):
        if len(self.bones.org.main) != 3:
            self.raise_error("Input to rig type must be a chain of 3 bones.")

        super().initialize()

    def prepare_bones(self):
        orgs = self.bones.org.main

        if self.params.rotation_axis == 'automatic':
            axis = compute_chain_x_axis(self.obj, orgs[0:2])

            for bone in orgs:
                align_bone_x_axis(self.obj, bone, axis)

        elif self.params.auto_align_extremity:
            axis = self.vector_without_z(self.get_bone(orgs[2]).z_axis)

            align_bone_z_axis(self.obj, orgs[2], axis)

    ####################################################
    # Settings

    @classmethod
    def parameters_ui(layout, params):
        super().parameters_ui(layout, params, 'Hand')
