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

from rigify.utils.naming import make_derived_name

from rigify.base_rig import stage

from ..spines import blenrig_spine
from . import spine_rigs


class Rig(spine_rigs.BaseBodyIkSpineRig, blenrig_spine.Rig):
    ####################################################
    # BONES
    #
    # mch:
    #   ik_forward_base
    #     First bone of the forward chain before hip IK correction
    #
    ####################################################

    def generate_body_ik_panel(self, panel):
        super().generate_body_ik_panel(panel)

        master = self.bones.ctrl.master
        self.make_property(master, 'body_ik_move_hips', 0.0, description='Body IK offsets the hips control instead of the whole spine')
        panel.custom_prop(master, 'body_ik_move_hips', text='Body IK Hips', slider=True)

        spine_rigs.add_spine_ik_snap(
            panel,
            master=self.bones.ctrl.hips,
            result=self.get_pre_hip_ik_result_bone(),
            final=self.bones.mch.ik_forward[0],
            text='Snap Hips to Hip IK',
        )

    def get_pre_hip_ik_result_bone(self):
        return self.bones.mch.ik_forward_base

    ####################################################
    # MCH IK forward chain

    @stage.generate_bones
    def make_mch_ik_forward_chain(self):
        super().make_mch_ik_forward_chain()

        self.bones.mch.ik_forward_base = self.make_mch_ik_forward_base_bone(self.bones.org[0])

    def make_mch_ik_forward_base_bone(self, org):
        return self.copy_bone(org, make_derived_name(org, 'mch', '.ik_forward_base'))

    @stage.parent_bones
    def parent_mch_ik_forward_chain(self):
        super().parent_mch_ik_forward_chain()

        base = self.bones.mch.ik_forward_base
        first = self.bones.mch.ik_forward[0]

        self.set_bone_parent(base, self.get_bone_parent(first))
        self.set_bone_parent(first, base)

    def rig_mch_ik_forward_bone(self, i, forward, back, tweak):
        if i == 0:
            super().rig_mch_ik_forward_bone(i, self.bones.mch.ik_forward_base, back, tweak)

            con = self.make_constraint(forward, 'COPY_LOCATION', self.bones.mch.hip_output)
            self.make_driver(con, 'influence', variables=[(self.bones.ctrl.master, 'body_ik_move_hips')])

        else:
            super().rig_mch_ik_forward_bone(i, forward, back, tweak)

    ####################################################
    # Hip offset bones

    def get_hip_offset_base_bone(self):
        return self.bones.mch.ik_forward[0]


def create_sample(obj):
    bones = blenrig_spine.create_sample(obj)
    pbone = obj.pose.bones[bones['spine']]
    pbone.rigify_type = 'body_ik.blenrig_spine'
