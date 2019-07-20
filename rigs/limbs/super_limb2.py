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

from rigify.base_generate import SubstitutionRig

from .limb_rigs import BaseLimbRig

from . import arm2 as arm
from . import leg2 as leg


class Rig(SubstitutionRig):
    def substitute(self):
        if self.params.limb_type == 'arm':
            return [ self.instantiate_rig(arm.Rig, self.base_bone) ]
        elif self.params.limb_type == 'leg':
            return [ self.instantiate_rig(leg.Rig, self.base_bone) ]
        elif self.params.limb_type == 'paw':
            return [ self.instantiate_rig('limbs.paw', self.base_bone) ]


def add_parameters(params):
    """ Add the parameters of this rig type to the
        RigifyParameters PropertyGroup
    """

    items = [
        ('arm', 'Arm', ''),
        ('leg', 'Leg', ''),
        ('paw', 'Paw', '')
    ]

    params.limb_type = bpy.props.EnumProperty(
        items   = items,
        name    = "Limb Type",
        default = 'arm'
    )

    BaseLimbRig.add_parameters(params)


def parameters_ui(layout, params):
    """ Create the ui for the rig parameters."""

    r = layout.row()
    r.prop(params, "limb_type")

    extremities = {'arm': 'Hand', 'leg': 'Foot', 'paw': 'Claw'}
    BaseLimbRig.parameters_ui(layout, params, extremities[params.limb_type])


def create_sample(obj):
    arm.create_sample(obj, limb=True)
