import bpy

from typing import Any, Callable, Iterable
from mathutils import Matrix

from rigify.utils.rig import connected_children_names, is_rig_base_bone
from rigify.utils.naming import make_derived_name
from rigify.utils.bones import BoneDict, align_bone_roll, align_bone_orientation, TypedBoneDict
from rigify.utils.mechanism import make_property
from rigify.utils.misc import map_list

from rigify.base_rig import stage, BaseRig, RigComponent
from itertools import count

from rigify.rigs.limbs import leg


class Rig(BaseRig):
    """Extra heel system for the leg rig."""

    def find_org_bones(self, bone):
        bones = BoneDict(
            main=[bone.name] + connected_children_names(self.obj, bone.name),
        )

        for b in self.get_bone(bones.main[0]).bone.children:
            if not b.use_connect and not b.children and not is_rig_base_bone(self.obj, b.name):
                bones.heel = b.name
                break
        else:
            self.raise_error("Heel bone not found.")

        return bones

    rigify_parent: leg.Rig

    leg_link: 'ExtraHeelLinkComponent'

    def initialize(self):
        orgs = self.bones.org.main

        if len(orgs) != 2:
            self.raise_error("Input to rig type must be a chain of 2 bones.")

        if not isinstance(self.rigify_parent, leg.Rig):
            self.raise_error("The extra heel must be a child of a leg rig.")

        if 'enabled' not in self.get_bone(self.base_bone):
            self.raise_error("The extra heel rig bone must have an 'enabled' custom property.")

        self.leg_link = self.instantiate_link()

    ####################################################
    # BONES

    class OrgBones(TypedBoneDict):
        main: list[str]                # Main bone chain
        heel: str                      # Heel bone

    class CtrlBones(BaseRig.CtrlBones):
        pass

    class MchBones(BaseRig.MchBones):
        heel: list[str]                # Chain of bones implementing foot roll.
        toe_ik_socket: str             # Dynamic socket for the toe in IK mode.
        toe_fk_socket: str             # Dynamic socket for the toe in FK mode.

    bones: BaseRig.ToplevelBones[
        'Rig.OrgBones',
        'Rig.CtrlBones',
        'Rig.MchBones',
        list[str]
    ]

    ####################################################
    # Initialization

    def instantiate_link(self):
        for item in self.rigify_parent.rigify_sub_objects:
            if isinstance(item, ExtraHeelLinkComponent):
                return item
        else:
            return ExtraHeelLinkComponent(self.rigify_parent)

    foot_scale: float
    foot_bend_matrix: Matrix
    toe_bend_matrix: Matrix
    toe_ik_socket_matrix: Matrix

    def prepare_bones(self):
        orgs = self.bones.org.main
        parent_orgs = self.rigify_parent.bones.org.main

        for org, parent_org in zip(orgs, parent_orgs[2:]):
            align_bone_roll(self.obj, org, parent_org)

        self.foot_scale = self.calc_foot_scale(orgs, parent_orgs)
        self.foot_bend_matrix = self.calc_foot_bend_matrix(orgs, parent_orgs)
        self.toe_bend_matrix = self.calc_toe_bend_matrix(orgs, parent_orgs)
        self.toe_ik_socket_matrix = self.calc_toe_ik_socket_matrix(orgs, parent_orgs)

    ####################################################
    # Utilities

    def get_enabled_ref(self):
        return self.bones.org.main[0], 'enabled'

    def calc_foot_scale(self, orgs: list[str], parent_orgs: list[str]):
        foot_bone = self.get_bone(orgs[0])
        parent_foot_bone = self.get_bone(parent_orgs[2])
        return foot_bone.length / parent_foot_bone.length

    def calc_foot_bend_matrix(self, orgs: list[str], parent_orgs: list[str]):
        foot_bone = self.get_bone(orgs[0])
        parent_foot_bone = self.get_bone(parent_orgs[2])
        return parent_foot_bone.matrix.inverted() @ foot_bone.matrix * self.foot_scale

    def calc_toe_bend_matrix(self, orgs: list[str], parent_orgs: list[str]):
        foot_bone = self.get_bone(orgs[0])
        toe_bone = self.get_bone(orgs[1])
        parent_foot_bone = self.get_bone(parent_orgs[2])
        parent_toe_bone = self.get_bone(parent_orgs[3])

        my_toe_bend = foot_bone.matrix.inverted() @ toe_bone.matrix
        adj_toe_bend = parent_toe_bone.matrix.inverted() @ parent_foot_bone.matrix @ my_toe_bend

        return adj_toe_bend * (1 / self.foot_scale)

    def calc_toe_ik_socket_matrix(self, orgs: list[str], parent_orgs: list[str]):
        toe_bone = self.get_bone(orgs[1])
        parent_toe_bone = self.get_bone(parent_orgs[3])
        roll_matrix = self.rigify_parent.roll_matrix.to_4x4()

        matrix = toe_bone.matrix @ parent_toe_bone.matrix.inverted() @ roll_matrix
        matrix.translation = toe_bone.head
        return matrix

    def align_to_parent_foot(self, name: str):
        bone = self.get_bone(name)
        pbone = self.get_bone(self.rigify_parent.bones.org.main[2])
        length = bone.length
        bone.head = pbone.head
        bone.tail = pbone.tail
        bone.length = length

    ####################################################
    # Heel mechanism

    def get_mch_heel_toe_output(self):
        return self.bones.mch.heel[-3]

    @stage.generate_bones
    def make_roll_mch_chain(self):
        orgs = self.bones.org.main
        heel = self.bones.org.heel
        self.bones.mch.heel = chain = self.make_roll_mch_bones(orgs[0], orgs[1], heel)
        parent_foot = self.rigify_parent.bones.org.main[2]
        self.align_roll_result_bone(chain[-1], orgs[0], parent_foot)

    def make_roll_mch_bones(self, foot: str, toe: str, heel: str):
        return self.rigify_parent.make_roll_mch_bones(foot, toe, heel)

    def align_roll_result_bone(self, name: str, _foot: str, parent_foot: str):
        align_bone_orientation(self.obj, name, parent_foot)
        # foot_bone = self.get_bone(foot)
        # parent_foot_bone = self.get_bone(parent_foot)
        # put_bone(self.obj, name, foot_bone.head, matrix=parent_foot_bone.matrix)

    @stage.parent_bones
    def parent_roll_mch_chain(self):
        chain = self.bones.mch.heel
        self.set_bone_parent(chain[0], self.rigify_parent.get_ik_pivot_output())
        self.parent_bone_chain(chain)

    @stage.rig_bones
    def rig_roll_mch_chain(self):
        ctrl = self.rigify_parent.bones.ctrl.heel
        chain = self.bones.mch.heel
        self.rig_roll_mch_bones(chain, ctrl, self.bones.org.heel)

        foot = self.bones.org.main[0]
        parent_foot = self.rigify_parent.bones.org.main[2]
        self.rotate_roll_result_bone(chain[-1], foot, parent_foot)

    def rig_roll_mch_bones(self, chain: list[str], ctrl_heel: str, org_heel: str):
        self.rigify_parent.rig_roll_mch_bones(chain, ctrl_heel, org_heel)

    def rotate_roll_result_bone(self, name: str, _foot: str, _parent_foot: str):
        bone = self.get_bone(name)
        bone.rotation_mode = 'QUATERNION'
        bone.matrix_basis = self.foot_bend_matrix
        bone.location = (0, 0, 0)

    ####################################################
    # Toe sockets

    @stage.generate_bones
    def make_toe_socket_bones(self):
        org_toe = self.bones.org.main[1]
        self.bones.mch.toe_fk_socket = self.copy_bone(
            org_toe, make_derived_name(org_toe, 'mch', '_fk_socket'), scale=0.5)
        self.bones.mch.toe_ik_socket = self.copy_bone(
            org_toe, make_derived_name(org_toe, 'mch', '_ik_socket'), scale=0.5)

        if not self.rigify_parent.use_ik_toe:
            self.get_bone(self.bones.mch.toe_ik_socket).matrix = self.toe_ik_socket_matrix

    @stage.parent_bones
    def parent_toe_socket_bones(self):
        self.set_bone_parent(self.bones.mch.toe_ik_socket, self.get_mch_heel_toe_output())

    @stage.rig_bones
    def rig_toe_socket_bones(self):
        bone = self.get_bone(self.bones.mch.toe_fk_socket)
        bone.matrix_basis = self.toe_bend_matrix
        bone.location = (0, 0, 0)

    ####################################################
    # ORG bones

    @stage.parent_bones
    def parent_org_chain(self):
        orgs = self.bones.org.main
        self.get_bone(orgs[1]).use_connect = False

    @stage.rig_bones
    def rig_org_chain(self):
        orgs = self.bones.org.main
        parent_orgs = self.rigify_parent.bones.org.main

        for i, org, parent_org in zip(count(0), orgs, parent_orgs[2:]):
            self.rig_org_bone(i, org, parent_org)

    def rig_org_bone(self, i: int, org: str, parent_org: str):
        self.make_constraint(org, 'COPY_TRANSFORMS', parent_org)

        if i == 0:
            self.make_constraint(
                org, 'COPY_SCALE', self.bones.mch.toe_fk_socket,
                space='LOCAL', use_offset=True,
            )

    ####################################################
    # DEF bones

    @stage.generate_bones
    def make_deform_chain(self):
        self.bones.deform = map_list(self.make_deform_bone, count(0), self.bones.org.main)

    def make_deform_bone(self, _i: int, org: str):
        return self.copy_bone(org, make_derived_name(org, 'def'), parent=False)

    @stage.parent_bones
    def parent_deform_chain(self):
        for deform, org in zip(self.bones.deform, self.bones.org.main):
            self.set_bone_parent(deform, org)

    @stage.rig_bones
    def rig_deform_chain(self):
        for i, deform, org in zip(count(0), self.bones.deform, self.bones.org.main):
            self.rig_deform_bone(i, deform, org)

    def rig_deform_bone(self, i: int, deform: str, _org: str):
        if i == 0:
            parent_def = self.rigify_parent.bones.deform[-2]
            self.make_constraint(deform, 'COPY_TRANSFORMS', parent_def)

            self.make_constraint(
                deform, 'COPY_SCALE', self.bones.mch.toe_fk_socket,
                space='LOCAL', use_offset=True
            )

            toe_org = self.bones.org.main[1]
            self.make_constraint(
                deform, 'STRETCH_TO', toe_org, keep_axis='SWING_Y',
                rest_length=self.get_bone(deform).bone.length,
            )

    ####################################################
    # Settings

    @classmethod
    def add_parameters(cls, params):
        pass

    @classmethod
    def parameters_ui(cls, layout, params):
        bone = bpy.context.active_pose_bone

        if bone and 'enabled' in bone:
            if type(bone['enabled']) is not int:
                layout.label(text="The 'enabled' property should be an integer.", icon='INFO')
            else:
                layout.label(text="The 'enabled' property exists.", icon='CHECKMARK')
        else:
            layout.label(text="The bone must have an 'enabled' custom property.", icon='ERROR')


class ExtraHeelLinkComponent(RigComponent):
    rigify_sub_object_run_late = True

    owner: leg.Rig

    heel_rigs: list[Rig]
    is_body_ik: bool

    toe_fk_socket: str
    toe_ik_socket: str

    def __init__(self, owner: leg.Rig):
        super().__init__(owner)

        self.heel_rigs = [rig for rig in owner.rigify_children if isinstance(rig, Rig)]

        from ..body_ik.limb_rigs import BaseBodyIkLimbRig

        if isinstance(self.owner, BaseBodyIkLimbRig):
            self.owner.use_middle_ik_parent_mch = True

    ####################################################
    # IK Widget Offset

    @stage.rig_bones
    def rig_ik_control_widget_offset(self):
        variables = {}
        expressions = []
        base_z = self.get_bone(self.owner.bones.org.heel).head.z

        for i, rig in enumerate(self.heel_rigs):
            offset = self.get_bone(rig.bones.org.heel).head.z - base_z
            variables[f"v{i}"] = rig.get_enabled_ref()
            expressions.append(f"{offset:.3f} if v{i}")

        expressions.append("0")

        ik_bone = self.get_bone(self.owner.bones.ctrl.ik)
        self.make_driver(ik_bone, 'custom_shape_translation', index=2,
                         variables=variables, expression=' else '.join(expressions))

    ####################################################
    # IK target

    @stage.rig_bones
    def rig_ik_target_bone(self):
        mch_target = self.owner.bones.mch.ik_target
        idx = find_index(
            self.get_bone(mch_target).constraints,
            lambda con: con.type == 'LIMIT_DISTANCE'
        )

        for rig in reversed(self.heel_rigs):
            self.rig_ik_target_link(rig, mch_target, idx)

    def rig_ik_target_link(self, rig: Rig, mch_target: str, idx: int):
        con = self.make_constraint(
            mch_target, 'COPY_TRANSFORMS', rig.bones.mch.heel[-1], insert_index=idx
        )
        self.make_driver(con, 'enabled', variables=[rig.get_enabled_ref()])

    ####################################################
    # FK foot

    @stage.rig_bones
    def rig_fk_foot_parent_bone(self):
        foot_parent = self.owner.bones.mch.fk[2]

        for rig in self.heel_rigs:
            self.rig_fk_foot_parent_link(rig, foot_parent)

    def rig_fk_foot_parent_link(self, rig: Rig, mch_parent: str):
        con = self.make_constraint(
            mch_parent, 'COPY_TRANSFORMS', rig.bones.mch.heel[-1],
            space='LOCAL', mix_mode='AFTER_SPLIT',
        )
        self.make_driver(con, 'enabled', variables=[rig.get_enabled_ref()])

    ####################################################
    # Toe

    @stage.generate_bones
    def make_toe_socket_bones(self):
        org_toe = self.owner.bones.org.main[3]
        self.toe_fk_socket = self.copy_bone(
            org_toe, make_derived_name(org_toe, 'mch', '_fk_socket'), scale=0.5)
        self.toe_ik_socket = self.copy_bone(
            org_toe, make_derived_name(org_toe, 'mch', '_ik_socket'), scale=0.5)

    @stage.parent_bones
    def parent_fk_toe_socket_bones(self):
        toe_out = self.owner.get_mch_heel_toe_output()
        self.set_bone_parent(self.toe_ik_socket, toe_out)

        if self.owner.use_ik_toe:
            self.set_bone_parent(self.owner.bones.ctrl.ik_toe, self.toe_ik_socket)
        else:
            align_bone_orientation(self.obj, self.toe_ik_socket, toe_out)

    @stage.rig_bones
    def rig_fk_toe_parent_bone(self):
        toe_parent = self.owner.bones.mch.fk[3]
        con_list = self.get_bone(toe_parent).constraints

        if not self.owner.use_ik_toe:
            out = self.owner.get_mch_heel_toe_output()
            idx = find_index(
                con_list,
                lambda con: con.type == 'COPY_TRANSFORMS' and con.subtarget == out
            )
            con_list[idx].subtarget = self.toe_ik_socket

        self.make_constraint(
            toe_parent, 'COPY_TRANSFORMS', self.toe_fk_socket,
            target_space='LOCAL_OWNER_ORIENT', owner_space='LOCAL',
            mix_mode='BEFORE_FULL', insert_index=0,
        )

        for rig in self.heel_rigs:
            self.rig_toe_fk_socket_link(rig, self.toe_fk_socket)

        for rig in self.heel_rigs:
            self.rig_toe_ik_socket_link(rig, self.toe_ik_socket)

    def rig_toe_fk_socket_link(self, rig: Rig, mch_socket: str):
        con = self.make_constraint(
            mch_socket, 'COPY_TRANSFORMS', rig.bones.mch.toe_fk_socket,
            space='LOCAL', mix_mode='AFTER_SPLIT',
        )
        self.make_driver(con, 'enabled', variables=[rig.get_enabled_ref()])

    def rig_toe_ik_socket_link(self, rig: Rig, mch_socket: str):
        con = self.make_constraint(
            mch_socket, 'COPY_TRANSFORMS', rig.bones.mch.toe_ik_socket,
        )
        self.make_driver(con, 'enabled', variables=[rig.get_enabled_ref()])

    ####################################################
    # Body IK

    @stage.rig_bones
    def rig_body_ik_parent_bones(self):
        from ..body_ik.limb_rigs import BaseBodyIkLimbRig

        if isinstance(self.owner, BaseBodyIkLimbRig):
            parents = self.owner.bones.mch.ik_mid_parents
            for i, parent in enumerate(parents):
                self.rig_body_ik_parent_bone(i, parent)

    def rig_body_ik_parent_bone(self, i: int, parent: str):
        # Foot parent
        if i == 0:
            parent_con_list = self.get_bone(parent).constraints

            # Remove redundant scale
            for con in list(parent_con_list):
                if con.type == 'COPY_SCALE' and\
                        con.subtarget in ('root', self.owner.bones.ctrl.master):
                    parent_con_list.remove(con)

            self.make_constraint(
                parent, 'COPY_TRANSFORMS', self.owner.bones.mch.fk[2],
                space='LOCAL', mix_mode='BEFORE_FULL',
            )

        # Toe parent
        elif i == 1:
            self.make_constraint(
                parent, 'COPY_TRANSFORMS', self.toe_fk_socket,
                space='LOCAL', mix_mode='BEFORE_FULL',
            )


def find_index(seq: Iterable[Any], test: Callable[[Any], bool]):
    for i, item in enumerate(seq):
        if test(item):
            return i
    else:
        return None


def create_sample(obj):
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('foot-extra.L')
    bone.head[:] = 0.0980, 0.0162, 0.0852
    bone.tail[:] = 0.0980, -0.0934, 0.0167
    bone.roll = 0.0000
    bones['foot-extra.L'] = bone.name
    bone = arm.edit_bones.new('toe-extra.L')
    bone.head[:] = 0.0980, -0.0934, 0.0167
    bone.tail[:] = 0.0980, -0.1606, 0.0167
    bone.roll = -0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['foot-extra.L']]
    bones['toe-extra.L'] = bone.name
    bone = arm.edit_bones.new('heel-extra.L')
    bone.head[:] = 0.0600, 0.0459, 0.0000
    bone.tail[:] = 0.1400, 0.0459, 0.0000
    bone.roll = 0.0000
    bone.use_connect = False
    bone.parent = arm.edit_bones[bones['foot-extra.L']]
    bones['heel-extra.L'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['foot-extra.L']]
    pbone.rigify_type = 'limbs.extra_heel'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    make_property(pbone, 'enabled', default=0, min=0, max=1)
    pbone = obj.pose.bones[bones['toe-extra.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['heel-extra.L']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'

    bpy.ops.object.mode_set(mode='EDIT')
    for bone in arm.edit_bones:
        bone.select = False
        bone.select_head = False
        bone.select_tail = False
    for b in bones:
        bone = arm.edit_bones[bones[b]]
        bone.select = True
        bone.select_head = True
        bone.select_tail = True
        arm.edit_bones.active = bone
        if bcoll := arm.collections.active:
            bcoll.assign(bone)

    return bones
