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
from rigify.base_rig import stage

from rigify.rigs.spines import basic_spine
from . import spine_rigs


class Rig(spine_rigs.BaseBodyIkSpineRig, basic_spine.Rig):
    ####################################################
    # BONES
    #
    # mch:
    #   hip_input
    #     Hip position before hip IK
    #
    ####################################################

    @stage.generate_bones
    def make_hip_input_bone(self):
        org = self.bones.org[0]
        mch = self.bones.mch
        mch.hip_input = self.copy_bone(org, make_derived_name(org, 'mch', '.hip_input'), scale=0.3)

    @stage.parent_bones
    def parent_hip_input_bone(self):
        self.set_bone_parent(self.bones.mch.hip_input, self.fk_result.hips[0])

    def get_pre_hip_ik_result_bone(self):
        return self.bones.mch.hip_input


def create_sample(obj):
    bones = basic_spine.create_sample(obj)
    pbone = obj.pose.bones[bones['spine']]
    pbone.rigify_type = 'body_ik.basic_spine'
