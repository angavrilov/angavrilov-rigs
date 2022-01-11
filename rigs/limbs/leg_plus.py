import bpy

import re
import itertools
import bisect
import math

from mathutils import Vector

from rigify.utils.naming import make_derived_name
from rigify.utils.bones import put_bone
from rigify.utils.misc import map_list, map_apply

from rigify.base_rig import stage
from itertools import count, repeat

from rigify.rigs.limbs import leg


DEG_360 = math.pi * 2


class Rig(leg.Rig):
    """Human leg rig extensions."""

    def initialize(self):
        super().initialize()

        self.use_toe_roll = self.params.extra_toe_roll

    def add_ik_only_buttons(self, panel, rig_name):
        super().add_ik_only_buttons(panel, rig_name)

        if self.use_toe_roll:
            bone = self.bones.ctrl.heel

            self.make_property(
                bone, 'Toe_Roll', default=0.0,
                description='Roll forward from the tip of the toe'
            )

            panel.custom_prop(bone, 'Toe_Roll', text='Roll Forward On Toe', slider=True)

    def make_roll_mch_bones(self, foot, toe, heel):
        chain = super().make_roll_mch_bones(foot, toe, heel)

        if self.use_toe_roll:
            rock2, rock1, roll2, roll1, result = chain

            roll3 = self.copy_bone(toe, make_derived_name(heel, 'mch', '_roll3'), scale=0.3)

            toe_pos = Vector(self.get_bone(toe).tail)
            toe_pos.z = self.get_bone(roll2).head.z

            put_bone(self.obj, roll3, toe_pos, matrix=self.roll_matrix)

            return [ rock2, rock1, roll2, roll3, roll1, result ]

        return chain

    def rig_roll_mch_bones(self, chain, heel, org_heel):
        if self.use_toe_roll:
            rock2, rock1, roll2, roll3, roll1, result = chain

            con = self.make_constraint(roll3, 'COPY_ROTATION', heel, space='LOCAL', use_xyz=(True, False, True))
            self.make_driver(con, 'influence', variables=[(heel, 'Toe_Roll')])

            if self.main_axis == 'x':
                self.make_constraint(roll3, 'LIMIT_ROTATION', max_x=DEG_360, space='LOCAL')
            else:
                self.make_constraint(roll3, 'LIMIT_ROTATION', max_z=DEG_360, space='LOCAL')

            chain = [ rock2, rock1, roll2, roll1, result ]

        super().rig_roll_mch_bones(chain, heel, org_heel)

    ####################################################
    # Settings

    @classmethod
    def add_parameters(self, params):
        super().add_parameters(params)

        params.extra_toe_roll = bpy.props.BoolProperty(
            name='Toe Tip Roll',
            default=False,
            description="Generate a slider to pivot forward roll from the tip of the toe"
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, 'extra_toe_roll')

        super().parameters_ui(layout, params)


def create_sample(obj):
    bones = leg.create_sample(obj)
    pbone = obj.pose.bones[bones['thigh.L']]
    pbone.rigify_type = 'limbs.leg_plus'
