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
from math import sqrt
from bl_math import clamp

from rigify.utils.mechanism import quote_property, driver_var_transform
from rigify.utils.naming import make_derived_name, Side, SideZ, get_name_side
from rigify.utils.widgets_basic import create_circle_widget
from rigify.utils.misc import map_list, force_lazy, LazyRef

from rigify.base_rig import stage

from rigify.rigs.skin.skin_parents import ControlBoneParentOffset
from rigify.rigs.skin.skin_rigs import BaseSkinRig

from rigify.rigs.skin.basic_chain import Rig as BasicChainRig


class Rig(BaseSkinRig):
    """
    Parent controller rig that stretches its child chains according
    to its scale, interpreting them as a set of concentric (half-)ellipses.
    """

    def find_org_bones(self, bone):
        return bone.name

    def initialize(self):
        super().initialize()

        self.make_control = self.params.make_control
        self.squash_limit = self.params.skin_spread_squash
        self.use_rhombus = self.params.skin_spread_rhombus_correction

        if self.make_control:
            self.input_ref = LazyRef(self.bones.ctrl, 'master')
        else:
            self.input_ref = self.base_bone

        matrix = self.get_bone(self.base_bone).bone.matrix_local

        self.transform_orientation = matrix.to_quaternion()
        self.transform_space = matrix.inverted()

        self.chain_to_layer = None
        self.init_child_chains()

        self.loop_ratio_vars = None

    ####################################################
    # UTILITIES

    def is_corner_node(self, node):
        "Checks if this node is where two L/R child chains meet."
        siblings = [n for n in node.get_merged_siblings() if n.rig in self.child_chains]

        sides_x = set(n.name_split.side for n in siblings)

        return {Side.LEFT, Side.RIGHT}.issubset(sides_x)

    def get_node_z(self, node):
        "Compute Z coordinate of the node in the local space of the control."
        return (self.transform_space @ node.point).z

    def get_node_side(self, node):
        "Compute the Z side of the node in the local space of the control."
        return 1 if self.get_node_z(node) > 0 else 0

    ####################################################
    # Control Nodes

    def init_child_chains(self):
        # Use child Left/Right chains
        self.child_chains = [
            rig for rig in self.rigify_children
            if isinstance(rig, BasicChainRig) and get_name_side(rig.base_bone) != Side.MIDDLE
        ]

    def arrange_child_chains(self):
        if self.chain_to_layer is not None:
            return

        # Build lists of corner nodes for all child chains.
        corners = [[], []]

        for child in self.child_chains:
            for node in child.control_nodes:
                if self.is_corner_node(node):
                    side = self.get_node_side(node)

                    if node.merged_master not in corners[side]:
                        corners[side].append(node.merged_master)

        tops = sorted(corners[1], key=self.get_node_z)
        bottoms = sorted(corners[0], key=self.get_node_z, reverse=True)

        if len(tops) != len(bottoms):
            self.raise_error(
                "Corner counts differ: {} vs {}",
                [n.name for n in tops], [n.name for n in bottoms],
            )

        # Build a mapping of child chain to concentric layer index
        self.chain_to_layer = {}

        for i, top, bottom in zip(count(0), tops, bottoms):
            for node in top.get_merged_siblings() + bottom.get_merged_siblings():
                if node.rig in self.child_chains:
                    cur_layer = self.chain_to_layer.get(node.rig, i)

                    if cur_layer != i:
                        self.raise_error(
                            "Conflicting chain layer on {}: {} and {}",
                            node.rig.base_bone, i, cur_layer)

                    self.chain_to_layer[node.rig] = i

        for child in self.child_chains:
            if child not in self.chain_to_layer:
                self.raise_error("Could not determine chain layer on {}", child.base_bone)

        # Build a mapping for nodes to layer and collect coordinates
        self.node_layer = {}

        pts = [ [] for top in tops ]

        for child in self.child_chains:
            layer_id = self.chain_to_layer[child]

            for node in child.control_nodes:
                if self.node_layer.get(node.merged_master, layer_id) != layer_id:
                    self.raise_error(
                        "Conflicting node layer on {}: {} and {}",
                        node.name, layer_id, self.node_layer[node.merged_master])

                self.node_layer[node.merged_master] = layer_id
                pts[layer_id].append(self.transform_space @ node.point)

        # Compute concentric half-ellipse sizes
        self.layer_sizes = []

        for i, item in enumerate(pts):
            # Sizes in Z direction
            minz = min(pt.z for pt in item)
            maxz = max(pt.z for pt in item)
            assert minz < 0 < maxz

            # Size in X direction: use points with smallest absolute Z
            minx = min((pt for pt in item if pt.x < 0), key=lambda p: abs(p.z)).x
            maxx = min((pt for pt in item if pt.x > 0), key=lambda p: abs(p.z)).x
            assert minx < 0 < maxx

            width = (abs(minx) + abs(maxx))/2

            self.layer_sizes.append((width, (-minz, maxz)))

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

    @stage.generate_widgets
    def make_master_control_widget(self):
        if self.make_control:
            ctrl = self.bones.ctrl.master
            bone = self.get_bone(ctrl)

            layer_x, layer_z = self.layer_sizes[0]
            radius = min(abs(x) for x in [layer_x, *layer_z])

            create_circle_widget(self.obj, ctrl, radius=radius/self.get_bone(ctrl).length)

    ####################################################
    # Scale mechanism

    def scale_expr(self, r_ratio, scale_expr, z_side, gap_scale=1):
        "Expression going from value 1 and derivative 0 at scale_expr=1, to asymptote scale_expr*r_ratio + C"
        squash = (1 - self.squash_limit[z_side]) * gap_scale * 2 / math.pi
        squash_expr = f'{squash:.4}*atan(({scale_expr}-1)*{r_ratio/(1-r_ratio)/squash:.4})'
        return f'lerp(1-{squash_expr},{scale_expr},{r_ratio:.4})'

    def arrange_scale_properties(self):
        self.loop_ratio_vars = [['','','']]

        self.auxvar_list_sz = []
        self.auxvar_list_sx = []

        # Compute the innermost ellipse size
        layer0_width, layer0_height = self.layer_sizes[0]

        self.inner_size = size = (layer0_width, sum(layer0_height)/2)

        # Determine if compensation is needed for the innermost larger dimension
        self.inner_radius = r = min(size)

        isround = r/max(size) > 0.9 or not self.params.skin_spread_inner_circle

        self.sxexpr = sxexpr = '$sx' if isround or size[0] == r else '$sv'
        self.szexpr = szexpr = '$sz' if isround or size[1] == r else '$sv'

        # Compute auxiliary variables
        for i, (width, zlim) in enumerate(self.layer_sizes[1:]):
            lvars = ['', '', '']

            # Ratios between current outer and innermost layer radius in the given dimension for rhombic correction
            if self.use_rhombus:
                if zlim[0] >= width * 1.1:
                    zr_ratio0 = size[1] / zlim[0]
                    lzexpr0 = f'max(1.001,{self.scale_expr(zr_ratio0, szexpr, 1)}/max(1e-3,{szexpr}*{zr_ratio0:.4}))'
                    lzexpr0 = lzexpr0.replace('$','')

                    lvars[0] = var = 'lz'+str(i)
                    self.auxvar_list_sz.append((var, lzexpr0))

                if zlim[1] >= width * 1.1:
                    zr_ratio1 = size[1] / zlim[1]
                    lzexpr1 = f'max(1.001,{self.scale_expr(zr_ratio1, szexpr, 1)}/max(1e-3,{szexpr}*{zr_ratio1:.4}))'
                    lzexpr1 = lzexpr1.replace('$','')

                    if lvars[0] and lzexpr0 == lzexpr1:
                        lvars[1] = lvars[0]
                    else:
                        lvars[1] = var = 'lz'+str(i)+'m'
                        self.auxvar_list_sz.append((var, lzexpr1))

                if width >= min(zlim) * 1.1:
                    xr_ratio = size[0] / width
                    lxexpr = f'max(1.001,{self.scale_expr(xr_ratio, sxexpr, 1)}/max(1e-3,{sxexpr}*{xr_ratio:.4}))'
                    lxexpr = lxexpr.replace('$','')

                    lvars[2] = var = 'lx'+str(i)
                    self.auxvar_list_sx.append((var, lxexpr))

            self.loop_ratio_vars.append(lvars)

    @stage.configure_bones
    def make_scale_properties(self):
        bone = self.get_bone(self.bones.org)

        input_bone = force_lazy(self.input_ref)

        svarsx = { 'sx': driver_var_transform(self.obj, input_bone, type='SCALE_X', space='LOCAL') }
        svarsz = { 'sz': driver_var_transform(self.obj, input_bone, type='SCALE_Z', space='LOCAL') }

        # If the innermost loop is not a circle, generate a proxy variable for the larger dimension
        fcu = None

        if self.sxexpr == '$sv':
            # Inner X dimension is larger than Z
            bone['sv'] = 0.0
            ratio = self.inner_size[1]/self.inner_size[0]
            fcu = self.make_driver(bone, quote_property('sv'), expression=f'sx*{ratio}', variables=svarsx)
            svarsx = { 'sv': (self.bones.org, 'sv') }

        elif self.szexpr == '$sv':
            # Inner Z dimension is larger than X
            bone['sv'] = 0.0
            ratio = self.inner_size[0]/self.inner_size[1]
            fcu = self.make_driver(bone, quote_property('sv'), expression=f'sz*{ratio}', variables=svarsz)
            svarsz = { 'sv': (self.bones.org, 'sv') }

        if fcu:
            # Add curve points for smooth transition from constant 1 to identity at 1
            fcu.extrapolation = 'LINEAR'
            kf = fcu.keyframe_points.insert(ratio, 1)
            kf.handle_left_type = kf.handle_right_type = 'ALIGNED'
            kf.handle_left = (0, 1)
            kf.handle_right = (1-(1-ratio)*0.25, 1)
            kf = fcu.keyframe_points.insert(1.1, 1.1)
            kf.handle_left_type = kf.handle_right_type = 'ALIGNED'
            kf.handle_left = (1, 1)
            kf.handle_right = (1.2, 1.2)

        # Emit auxiliary variables
        for var, expr in self.auxvar_list_sx:
            bone[var] = 1.0
            self.make_driver(bone, quote_property(var), expression=expr, variables=svarsx)

        for var, expr in self.auxvar_list_sz:
            bone[var] = 1.0
            self.make_driver(bone, quote_property(var), expression=expr, variables=svarsz)

    def rhombic_scale_expr(self, sxexpr, pt_x, pt_z, in_xsize, in_zsize, out_xsize, out_zsize, dim_idx):
        """
        Apply correction to round out loops that have rhombic rather than elliptical shape.
        sxexpr: input X scale expression
        pt_x, pt_z: point coordinates
        in_xsize, in_zsize: inner loop size
        out_xsize, out_zsize: outer loop size
        dim_idx: dimension
        """
        zpos = abs(pt_z) / out_zsize
        common_sexpr = self.scale_expr(in_xsize/out_xsize, sxexpr, dim_idx)

        if zpos < 1:
            l = out_zsize / in_zsize
            sxsize = -out_xsize if pt_x < 0 else out_xsize

            # Circle transitioning into line from (1,0) to (0,l) at rest and in current pose
            xbaseval = (sqrt(1-pow(clamp(zpos*l),2)) if zpos*l*l < 1 else (1-zpos)*l/sqrt(l*l-1))
            xbase = f'(sqrt(1-pow(clamp({zpos:.3}*$l),2)) if {zpos:.3}*$l*$l < 1 else {1-zpos:.3}*$l/sqrt($l*$l-1))'

            # Pure ellipse coordinate
            ellipse_x = sqrt(1-pow(clamp(zpos),2))

            # Fade correction depending on how close the actual shape is to rhombus or ellipse
            fac = clamp((ellipse_x - abs(pt_x/out_xsize))/max(1e-6, ellipse_x - xbaseval))

            if fac > 0:
                return f'({common_sexpr})*({pt_x:.4}+({xbase}-{xbaseval:.4})*{sxsize*fac:.4})-{pt_x:.4}'

        return f'({common_sexpr}-1)*{pt_x:.4}'

    def extend_control_node_parent(self, parent, node):
        self.arrange_child_chains()

        if node.merged_master not in self.node_layer:
            return parent

        if not self.loop_ratio_vars:
            self.arrange_scale_properties()

        # Prepare parameters
        parent = ControlBoneParentOffset(self, node, parent)

        layer = self.node_layer[node.merged_master]
        layer_width, layer_limz = self.layer_sizes[layer]

        pt = self.transform_space @ node.point
        side = self.get_node_side(node)

        svars = {
            'sx': driver_var_transform(self.obj, self.input_ref, type='SCALE_X', space='LOCAL'),
            'sy': driver_var_transform(self.obj, self.input_ref, type='SCALE_Y', space='LOCAL'),
            'sz': driver_var_transform(self.obj, self.input_ref, type='SCALE_Z', space='LOCAL'),
            'sv': (self.bones.org, 'sv'),
        }

        # Scale based offsets
        yoff = f'($sy-1)*{pt.y:.4}'

        if layer == 0:
            # Innermost loop
            xoff = f'({self.sxexpr}-1)*{pt.x:.4}'
            zoff = f'({self.szexpr}-1)*{pt.z:.4}'

        else:
            zlimit = layer_limz[side]

            xr_ratio = self.inner_size[0] / layer_width
            zr_ratio = self.inner_size[1] / zlimit

            xoff = f'({self.scale_expr(xr_ratio, self.sxexpr, 0)}-1)*{pt.x:.4}'
            zoff = f'({self.scale_expr(zr_ratio, self.szexpr, 1)}-1)*{pt.z:.4}'

            # Apply circle+line shape correction for asymmetric loops
            if self.use_rhombus:
                if zlimit > layer_width * 1.1:
                    svars['l'] = (self.bones.org, self.loop_ratio_vars[layer][side])

                    xoff = self.rhombic_scale_expr(
                        self.sxexpr, pt.x, pt.z, self.inner_size[0], self.inner_size[1], layer_width, zlimit, 0)

                elif layer_width > zlimit * 1.1:
                    svars['l'] = (self.bones.org, self.loop_ratio_vars[layer][2])

                    zoff = self.rhombic_scale_expr(
                        self.szexpr, pt.z, pt.x, self.inner_size[1], self.inner_size[0], zlimit, layer_width, 1)

        parent.add_location_driver(self.transform_orientation, 0, xoff, svars)
        parent.add_location_driver(self.transform_orientation, 1, yoff, svars)
        parent.add_location_driver(self.transform_orientation, 2, zoff, svars)

        parent.add_copy_local_location(self.input_ref, influence=self.params.skin_spread_fade**layer)
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

        params.skin_spread_squash = bpy.props.FloatVectorProperty(
            name="Squash Limit", size=2,
            default=[0.1, 0.1], min=0, max=1,
            description="Specifies how small each gap between loops can be squashed"
        )

        params.skin_spread_fade = bpy.props.FloatProperty(
            name="Layer Fade", default=0.5, min=0, max=1,
            description="Specifies how much the influence of the control translation fades for each loop",
        )

        params.skin_spread_inner_circle = bpy.props.BoolProperty(
            name="Circularize Inner Shape", default=False,
            description="If the inner loop isn't circular, delay upscale of the longer dimension until it is circularized",
        )

        params.skin_spread_rhombus_correction = bpy.props.BoolProperty(
            name="Rhombus Correction", default=True,
            description="Apply correction to widen loops that have rhombic rather than elliptical shape into ellipses",
        )

    @classmethod
    def parameters_ui(self, layout, params):
        layout.prop(params, "make_control", text="Generate Control")

        row = layout.row()
        row.prop(params, "skin_spread_squash")

        layout.prop(params, "skin_spread_fade")
        layout.prop(params, "skin_spread_inner_circle")
        layout.prop(params, "skin_spread_rhombus_correction")

