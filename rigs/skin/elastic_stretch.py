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

from itertools import count, repeat
from mathutils import Vector, Matrix

from rigify.utils.rig import connected_children_names
from rigify.utils.layers import ControlLayersOption
from rigify.utils.mechanism import quote_property, driver_var_transform
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import align_bone_orientation, align_bone_to_axis, align_bone_roll
from rigify.utils.widgets_basic import create_circle_widget
from rigify.utils.misc import map_list

from rigify.base_rig import stage

from .skin_rigs import BaseSkinRig, ControlBoneParentOffset, LazyRef


class Rig(BaseSkinRig):
    """Elastic deform scale/pinch brush control."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        bone = self.get_bone(self.base_bone)
        matrix = bone.bone.matrix_local

        self.transform_orientation = matrix.to_quaternion()
        self.transform_space = matrix.inverted()

    def get_control_node_rotation(self):
        return self.transform_orientation

    def build_control_node_parent(self, node, parent_bone):
        return self.build_control_node_parent_next(node)

    def get_child_chain_parent(self, rig, parent_bone):
        return self.get_child_chain_parent_next(rig)

    ####################################################
    # BONES
    #
    # ctrl:
    #   master
    #     Master control
    #
    ####################################################

    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control(self):
        self.bones.ctrl.master = self.copy_bone(self.bones.org, make_derived_name(self.bones.org, 'ctrl'), parent=True)

    @stage.configure_bones
    def configure_master_control(self):
        self.copy_bone_properties(self.bones.org, self.bones.ctrl.master)

        # Lock Y scale since it's not used
        bone = self.get_bone(self.bones.ctrl.master)
        bone.lock_scale[1] = True
        bone.lock_rotation = (True, True, True)
        bone.lock_rotation_w = True

    @stage.rig_bones
    def rig_master_control(self):
        # Scaling above 10x causes folds to form
        self.make_constraint(
            self.bones.ctrl.master, 'LIMIT_SCALE',
            max_x=10, max_y=10, max_z=10, owner_space='LOCAL',
            use_transform_limit=True,
        )

    @stage.generate_widgets
    def make_master_control_widget(self):
        ctrl = self.bones.ctrl.master
        radius = self.params.skin_elastic_scale_radius
        bone = self.get_bone(ctrl)

        create_circle_widget(self.obj, ctrl, radius=radius/self.get_bone(ctrl).length)

    ####################################################
    # Scale mechanism

    @stage.configure_bones
    def make_scale_properties(self):
        org = self.bones.org
        bone = self.get_bone(org)
        ctrl = self.bones.ctrl.master

        bone['s'] = 0.0
        bone['p'] = 0.0
        bone['g'] = 0.0

        variables = {
            'sx': driver_var_transform(self.obj, self.bones.ctrl.master, type='SCALE_X', space='LOCAL'),
            'sy': driver_var_transform(self.obj, self.bones.ctrl.master, type='SCALE_Z', space='LOCAL'),
        }

        self.make_driver(bone, quote_property('s'), expression='sx+sy-2', variables=variables)
        self.make_driver(bone, quote_property('p'), expression='sx-sy', variables=variables)

        variables = {
            'tx': driver_var_transform(self.obj, self.bones.ctrl.master, type='LOC_X', space='LOCAL'),
            'ty': driver_var_transform(self.obj, self.bones.ctrl.master, type='LOC_Z', space='LOCAL'),
        }

        self.make_driver(bone, quote_property('g'), expression='sqrt(tx*tx+ty*ty)', variables=variables)

    def extend_control_node_parent(self, parent, node):
        parent = ControlBoneParentOffset.wrap(self, parent, node)

        pos = self.transform_space @ node.point

        # Compute brush parameters
        radius = self.params.skin_elastic_scale_radius
        poissons_ratio = 0.3

        mat1 = compute_scale_pinch_matrix(pos.x, pos.z, radius, poissons_ratio, EPS_MIN)
        mat2 = compute_scale_pinch_matrix(pos.x, pos.z, radius, poissons_ratio, 5)

        trf_weight1 = compute_translate_weight(pos.x, pos.z, radius, EPS_MIN)
        trf_weight2 = compute_translate_weight(pos.x, pos.z, radius, 5)

        # Apply scale & pinch drivers
        variables = {
            's': (LazyRef(self.bones, 'org'), 's'),
            'p': (LazyRef(self.bones, 'org'), 'p'),
            'g': (LazyRef(self.bones, 'org'), 'g'),
        }

        # Solution for the first minimum touching the 0.1 compression plane at 10x scale
        max_blend = 20
        expt = 0.5765756146
        coeff = 0.2359836090 / (max_blend ** expt)

        expr_x = 'lerp(%f*$s+%f*$p,%f*$s+%f*$p,%f*pow(clamp($s,0,%d),%f))' % (
            mat1[0][0], mat1[0][1], mat2[0][0], mat2[0][1], coeff, max_blend, expt)
        expr_y = 'lerp(%f*$s+%f*$p,%f*$s+%f*$p,%f*pow(clamp($s,0,%d),%f))' % (
            mat1[1][0], mat1[1][1], mat2[1][0], mat2[1][1], coeff, max_blend, expt)

        parent.add_location_driver(self.transform_orientation, 0, expr_x, variables)
        parent.add_location_driver(self.transform_orientation, 2, expr_y, variables)

        # Add translate control for center (not a true grab brush, just falloff matching scale)
        ctrl = LazyRef(self.bones.ctrl, 'master')

        if trf_weight1 >= 1 and trf_weight2 >= 1:
            parent.add_copy_local_location(ctrl)
        else:
            expr_i = 'lerp(%f,%f,%f*pow(clamp($s+2*$g,0,%d),%f))' % (
                trf_weight1, trf_weight2, coeff, max_blend, expt)

            parent.add_copy_local_location(ctrl, influence_expr=expr_i, influence_vars=variables)

        return parent

    ####################################################
    # ORG bone

    @stage.rig_bones
    def rig_org_bone(self):
        pass

    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        params.skin_elastic_scale_radius = bpy.props.FloatProperty(
            name='Exact Scale Radius',
            default=1,
            min=0,
            description='Radius at which the control scale is applied exactly'
        )

    @classmethod
    def parameters_ui(self, layout, params):
        r = layout.row()
        r.prop(params, "skin_elastic_scale_radius")


# This value gives maximum scale offset at exactly radius (radius ring radially unscaled)
EPS_MIN = 0.5*math.sqrt(3 + math.sqrt(17))

def compute_scale_pinch_matrix(x, y, exact_radius, poissons_ratio, brush_radius):
    x /= exact_radius
    y /= exact_radius
    v = poissons_ratio
    eps = brush_radius

    x2 = x * x
    y2 = y * y
    e2 = eps * eps

    common = (e2 + 1) / (e2 + x2 + y2)
    common *= common

    common_scale = common * (2*e2 + x2 + y2) / 2 / (2*e2 + 1)
    common_pinch = common / (4*e2*v - 3*e2 + 2*v - 2)

    v2_32e2_x2y2v = (2*v - 1.5) * e2 + (x2 + y2) * v

    return exact_radius * Matrix((
        ( common_scale * x, common_pinch * x * (v2_32e2_x2y2v - x2) ),
        ( common_scale * y, common_pinch * -y * (v2_32e2_x2y2v - y2) ),
    ))

def compute_translate_weight(x, y, exact_radius, brush_radius):
    x /= exact_radius
    y /= exact_radius

    r2 = x*x + y*y
    r = math.sqrt(r2)

    if r <= 1:
        return 1

    e2 = brush_radius * brush_radius

    return r * (e2 + 1) * (e2 + 1) * (2*e2 + r2) / (2*e2 + 1) / (e2 + r2) / (e2 + r2)
