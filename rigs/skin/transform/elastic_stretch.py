# ====================== BEGIN GPL LICENSE BLOCK ======================
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
# ======================= END GPL LICENSE BLOCK ========================

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
from rigify.utils.misc import map_list, force_lazy, LazyRef

from rigify.base_rig import stage

from ..skin_parents import ControlBoneParentOffset
from ..skin_rigs import BaseSkinRig


class Rig(BaseSkinRig):
    """Elastic deform scale/pinch brush control."""

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.make_control = self.params.make_control

        if self.make_control:
            self.input_ref = LazyRef(self.bones.ctrl, 'master')
        else:
            self.input_ref = self.base_bone

        bone = self.get_bone(self.base_bone)
        matrix = bone.bone.matrix_local

        self.transform_orientation = matrix.to_quaternion()
        self.transform_space = matrix.inverted()

    ####################################################
    # Control Nodes

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
        if self.make_control:
            self.bones.ctrl.master = self.copy_bone(
                self.bones.org, make_derived_name(self.bones.org, 'ctrl'), parent=True)

    @stage.configure_bones
    def configure_master_control(self):
        if self.make_control:
            self.copy_bone_properties(self.bones.org, self.bones.ctrl.master)

            # Lock Y scale since it's not used
            bone = self.get_bone(self.bones.ctrl.master)
            bone.lock_scale[1] = True
            bone.lock_rotation = (True, True, True)
            bone.lock_rotation_w = True

    @stage.rig_bones
    def rig_master_control(self):
        if self.make_control:
            # Scaling above this causes folds to form
            self.make_constraint(
                self.bones.ctrl.master, 'LIMIT_SCALE',
                max_x=self.max_scale, max_y=self.max_scale, max_z=self.max_scale, owner_space='LOCAL',
                use_transform_limit=True,
            )

    @stage.generate_widgets
    def make_master_control_widget(self):
        if self.make_control:
            ctrl = self.bones.ctrl.master
            radius = self.params.skin_elastic_scale_radius
            bone = self.get_bone(ctrl)

            create_circle_widget(self.obj, ctrl, radius=radius/self.get_bone(ctrl).length)

    ####################################################
    # Scale mechanism

    @stage.initialize
    def init_scale_params1(self):
        self.eps = EPS_MIN
        self.k_list = [1, 3.55, 11]  # x 1.9, 3.5
        self.blends = [
            '(-2.381*$f-20.883*atan($f*0.29)+17.191*atan($f*0.491))/(4.976*atan($f*0.491)-13.578*atan($f*0.29)-11.655*$f)',
            '(-1.033*atan($f*0.29)+0.378*atan($f*0.49)+0.114*$f)/$f',
        ]
        self.max_scale = 35

    # @stage.initialize
    def init_scale_params2(self):
        self.eps = 1
        self.k_list = [1, 3.6, 12]  # x 1.5, 2.5
        self.blends = [
            '(-2.654*$f+11.758*atan($f*0.485)-15.159*atan($f*0.378))/(-9.938*$f+2.868*atan($f*0.485)-10.105*atan($f*0.378))',
            '(0.134*$f+0.250*atan($f*0.485)-0.881*atan($f*0.378))/$f',
        ]
        self.max_scale = 30

    @stage.configure_bones
    def make_scale_properties(self):
        org = self.bones.org
        bone = self.get_bone(org)
        input_bone = force_lazy(self.input_ref)

        bone['s'] = 0.0
        bone['p'] = 0.0
        bone['f'] = 0.0

        variables = {
            'sx': driver_var_transform(self.obj, input_bone, type='SCALE_X', space='LOCAL'),
            'sy': driver_var_transform(self.obj, input_bone, type='SCALE_Z', space='LOCAL'),
        }

        self.make_driver(bone, quote_property('s'), expression='sx+sy-2', variables=variables)
        self.make_driver(bone, quote_property('p'), expression='sx-sy', variables=variables)

        variables = {
            **variables,
            'tx': driver_var_transform(self.obj, input_bone, type='LOC_X', space='LOCAL'),
            'ty': driver_var_transform(self.obj, input_bone, type='LOC_Z', space='LOCAL'),
        }

        self.make_driver(bone, quote_property('f'),
                         expression='max(1e-5,(sx+sy-2)/2+sqrt(tx*tx+ty*ty))',
                         variables=variables)

    def extend_control_node_parent(self, parent, node):
        parent = ControlBoneParentOffset(self, node, parent)

        pos = self.transform_space @ node.point

        # Compute brush parameters
        radius = self.params.skin_elastic_scale_radius
        poissons_ratio = 0.3

        mats = [compute_scale_pinch_matrix(pos.x, pos.z, radius, poissons_ratio, self.eps * k)
                for k in self.k_list]
        trf_weights = [compute_translate_weight(pos.x, pos.z, radius, self.eps * k)
                       for k in self.k_list]

        # Apply scale & pinch drivers
        variables = {
            's': (LazyRef(self.bones, 'org'), 's'),
            'p': (LazyRef(self.bones, 'org'), 'p'),
            'f': (LazyRef(self.bones, 'org'), 'f'),
        }

        exprs_x = ['%f*$s+%f*$p' % (mat[0][0], mat[0][1]) for mat in mats]
        exprs_y = ['%f*$s+%f*$p' % (mat[1][0], mat[1][1]) for mat in mats]

        parent.add_location_driver(self.transform_orientation, 0,
                                   lerp_mix(exprs_x, self.blends), variables)
        parent.add_location_driver(self.transform_orientation, 2,
                                   lerp_mix(exprs_y, self.blends), variables)

        # Add translate control for center (not a true grab brush, just falloff matching scale)
        if all(w >= 1 for w in trf_weights):
            parent.add_copy_local_location(self.input_ref)
        else:
            expr_i = lerp_mix(map(str, trf_weights), self.blends)

            parent.add_copy_local_location(
                self.input_ref, influence_expr=expr_i, influence_vars=variables)

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
        params.make_control = bpy.props.BoolProperty(
            name="Control",
            default=True,
            description="Create a control bone for the copy"
        )

        params.skin_elastic_scale_radius = bpy.props.FloatProperty(
            name='Exact Scale Radius',
            default=1,
            min=0,
            description='Radius at which the control scale is applied exactly'
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "make_control", text="Generate Control")
        layout.prop(params, "skin_elastic_scale_radius")


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
        (common_scale * x, common_pinch * x * (v2_32e2_x2y2v - x2)),
        (common_scale * y, common_pinch * -y * (v2_32e2_x2y2v - y2)),
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


def lerp_mix(items, weights):
    cur, *rest = items
    for item, weight in zip(rest, weights):
        cur = 'lerp(%s,%s,%s)' % (cur, item, weight)
    return cur
