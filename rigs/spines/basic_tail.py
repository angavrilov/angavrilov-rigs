import bpy

from itertools import count

from rigify.utils.naming import strip_org, make_derived_name
from rigify.utils.bones import put_bone, flip_bone, is_same_position, connect_bbone_chain_handles, align_bone_orientation
from rigify.utils.widgets_basic import create_circle_widget
from rigify.utils.layers import ControlLayersOption
from rigify.utils.misc import map_list

from rigify.base_rig import stage
from rigify.rigs.chain_rigs import TweakChainRig, SimpleChainRig

from rigify.rigs.widgets import create_ballsocket_widget


class Rig(TweakChainRig):
    bbone_segments = 8

    def initialize(self):
        super(Rig,self).initialize()

        self.copy_rotation_axes = self.params.copy_rotation_axes

        self.use_connect = self.params.connect_chain
        self.use_connect_master = False
        self.use_connect_tweak = False

        if self.use_connect:
            parent = self.rigify_parent

            if not isinstance(parent, SimpleChainRig):
                raise MetarigError("RIGIFY ERROR: Bone '%s': cannot connect to non-chain parent rig." % (strip_org(self.base_bone)))

            if not is_same_position(self.obj, parent.bones.org[0], self.bones.org[0]):
                raise MetarigError("RIGIFY ERROR: Bone '%s': cannot connect chain - bone position is disjoint." % (strip_org(self.base_bone)))


    def prepare_bones(self):
        orgs = self.bones.org

        if self.use_connect:
            # Exactly match bone position to parent
            self.get_bone(orgs[0]).head = self.get_bone(self.rigify_parent.bones.org[0]).head

    ####################################################
    # Utilities

    def get_rig_parent(self):
        if self.use_connect_tweak:
            return self.bones.ctrl.tweak[0]
        else:
            return self.get_bone_parent(self.bones.org[0])

    ####################################################
    # Master control

    @stage.generate_bones
    def make_master_control_bone(self):
        org = self.bones.org[0]
        self.bones.ctrl.master = self.copy_bone(org, make_derived_name(org, 'ctrl', '_master'))

    @stage.parent_bones
    def parent_master_control_bone(self):
        self.set_bone_parent(self.bones.ctrl.master, self.get_rig_parent())

    @stage.generate_widgets
    def make_master_control_widget(self):
        bone = self.bones.ctrl.master
        self.get_bone(bone).custom_shape_transform = self.get_bone(self.bones.ctrl.tweak[-1])
        create_ballsocket_widget(self.obj, bone, size=0.7)

    ####################################################
    # Control bones

    @stage.parent_bones
    def parent_control_chain(self):
        self.set_bone_parent(self.bones.ctrl.fk[0], self.bones.mch.rot_tail)
        self.parent_bone_chain(self.bones.ctrl.fk, use_connect=False)

    @stage.rig_bones
    def rig_control_chain(self):
        ctrls = self.bones.ctrl.fk
        for args in zip(ctrls, [self.bones.ctrl.master] + ctrls):
            self.rig_control_bone(*args)

    def rig_control_bone(self, ctrl, prev_ctrl):
        self.make_constraint(
            ctrl, 'COPY_ROTATION', prev_ctrl,
            use_xyz=self.copy_rotation_axes,
            space='LOCAL', use_offset=True
        )

    # Widgets
    def make_control_widget(self, ctrl):
        create_circle_widget(self.obj, ctrl, radius=0.5, head_tail=0.75)

    ####################################################
    # MCH bones associated with main controls

    @stage.generate_bones
    def make_mch_control_bones(self):
        self.bones.mch.rot_tail = self.make_mch_rotation_bone(self.bones.org[0], 'ROT-tail')

    def make_mch_rotation_bone(self, org, name):
        return self.copy_bone(org, make_derived_name(name, 'mch'), parent=True)

    @stage.parent_bones
    def align_mch_rotation_bones(self):
        # Choose which bone to follow
        mch = self.bones.mch
        parent = self.rigify_parent

        if self.use_connect and 'master' in parent.bones.ctrl:
            self.use_connect_master = True
            self.follow_bone = parent.bones.ctrl.master
        else:
            self.follow_bone = 'root'

        # Align rot bone orientations to it
        align_bone_orientation(self.obj, mch.rot_tail, self.follow_bone)

    @stage.parent_bones
    def parent_mch_control_bones(self):
        self.set_bone_parent(self.bones.mch.rot_tail, self.get_rig_parent())

    @stage.configure_bones
    def configure_follow_properties(self):
        # Choose where to put custom properties
        controls = self.bones.ctrl.flatten()

        if self.use_connect_master:
            owner = self.rigify_parent
            self.prop_bone = self.follow_bone
            controls += self.rigify_parent.bones.ctrl.flatten()
        else:
            owner = self
            self.prop_bone = self.ctrl.master

        # Generate properties and script
        self.make_property(self.prop_bone, 'tail_follow', default=0.0)

        panel = self.script.panel_with_selected_check(owner, controls)
        panel.custom_prop(self.prop_bone, 'tail_follow', text='Tail Follow', slider=True)

    @stage.rig_bones
    def rig_mch_control_bones(self):
        self.rig_mch_rotation_bone(self.bones.mch.rot_tail, 'tail_follow')

    def rig_mch_rotation_bone(self, mch, prop_name):
        con = self.make_constraint(mch, 'COPY_ROTATION', self.follow_bone)
        self.make_constraint(mch, 'COPY_SCALE', self.follow_bone)

        self.make_driver(con, 'influence', variables=[(self.prop_bone, prop_name)], polynomial=[1,-1])


    ####################################################
    # Tweak bones

    @stage.generate_bones
    def make_tweak_chain(self):
        orgs = self.bones.org
        self.bones.ctrl.tweak = map_list(self.make_tweak_bone, count(0), orgs[0:1] + orgs)

    def make_tweak_bone(self, i, org):
        if i == 0:
            if self.use_connect and isinstance(self.rigify_parent, TweakChainRig):
                # Share the last tweak bone of the parent rig
                name = self.rigify_parent.bones.ctrl.tweak[0]

                if not is_same_position(self.obj, name, org):
                    raise MetarigError("RIGIFY ERROR: Bone '%s': cannot connect tweaks - position mismatch" % (strip_org(self.base_bone)))

                self.use_connect_tweak = True

            else:
                name = self.copy_bone(org, 'tweak_base_' + strip_org(org), parent=False, scale=0.5)

        else:
            name = self.copy_bone(org, 'tweak_' + strip_org(org), parent=False, scale=0.5)
            put_bone(self.obj, name, self.get_bone(org).tail)

        return name

    @stage.parent_bones
    def parent_tweak_chain(self):
        ctrl = self.bones.ctrl
        for i, tweak, main in zip(count(0), ctrl.tweak, ctrl.fk + ctrl.fk[-1:]):
            if i > 0 or not self.use_connect_tweak:
                self.set_bone_parent(tweak, main)

    @stage.configure_bones
    def configure_tweak_chain(self):
        super().configure_tweak_chain()

        ControlLayersOption.TWEAK.assign(self.params, self.obj, self.bones.ctrl.tweak)


    ##############################
    # ORG chain

    @stage.parent_bones
    def parent_org_chain(self):
        self.set_bone_parent(self.bones.org[0], self.bones.ctrl.tweak[0])

    def rig_org_bone(self, org, tweak, next_tweak):
        self.make_constraint(org, 'DAMPED_TRACK', next_tweak)
        self.make_constraint(org, 'STRETCH_TO', next_tweak)


    ##############################
    # Deform chain

    def make_deform_bone(self, org):
        if self.use_connect:
            name = self.copy_bone(org, make_derived_name(org, 'def'), parent=False)
            flip_bone(self.obj, name)
        else:
            name = self.copy_bone(org, make_derived_name(org, 'def'), parent=True)

        self.get_bone(name).bbone_segments = self.bbone_segments
        return name

    @stage.parent_bones
    def parent_deform_chain(self):
        if self.use_connect:
            for deform, org in zip(self.bones.deform, self.bones.org):
                self.set_bone_parent(deform, org)

            parent_def = self.rigify_parent.bones.deform[0]
            connect_bbone_chain_handles(self.obj, [ *reversed(self.bones.deform), parent_def ])

    @stage.configure_bones
    def configure_deform_chain(self):
        if self.use_connect:
            self.get_bone(self.bones.deform[-1]).bone.bbone_easein = 0.0
            self.get_bone(self.rigify_parent.bones.deform[0]).bone.bbone_easein = 1.0
        else:
            self.get_bone(self.bones.deform[-1]).bone.bbone_easeout = 0.0

    @stage.rig_bones
    def rig_deform_chain(self):
        if not self.use_connect:
            super().rig_deform_chain()


    ####################################################
    # SETTINGS

    @classmethod
    def add_parameters(self, params):
        """ Add the parameters of this rig type to the
            RigifyParameters PropertyGroup
        """

        params.connect_chain = bpy.props.BoolProperty(
            name='Connect chain',
            default=False,
            description='Connect the B-Bone chain to the parent rig'
        )

        params.copy_rotation_axes = bpy.props.BoolVectorProperty(
            size=3,
            description="Automation axes",
            default=tuple([i == 0 for i in range(0, 3)])
            )

        # Setting up extra tweak layers
        ControlLayersOption.TWEAK.add_parameters(params)


    @classmethod
    def parameters_ui(self, layout, params):
        """ Create the ui for the rig parameters.
        """

        r = layout.row()
        r.prop(params, "connect_chain")

        row = layout.row(align=True)
        for i, axis in enumerate(['x', 'y', 'z']):
            row.prop(params, "copy_rotation_axes", index=i, toggle=True, text=axis)

        ControlLayersOption.TWEAK.parameters_ui(layout, params)


def create_sample(obj):
    # generated by rigify.utils.write_metarig
    bpy.ops.object.mode_set(mode='EDIT')
    arm = obj.data

    bones = {}

    bone = arm.edit_bones.new('Bone')
    bone.head[:] = 0.0000, 0.0000, 0.0000
    bone.tail[:] = 0.0000, 0.0000, 0.3333
    bone.roll = 0.0000
    bone.use_connect = False
    bones['Bone'] = bone.name

    bone = arm.edit_bones.new('Bone.002')
    bone.head[:] = 0.0000, 0.0000, 0.3333
    bone.tail[:] = 0.0000, 0.0000, 0.6667
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['Bone']]
    bones['Bone.002'] = bone.name

    bone = arm.edit_bones.new('Bone.001')
    bone.head[:] = 0.0000, 0.0000, 0.6667
    bone.tail[:] = 0.0000, 0.0000, 1.0000
    bone.roll = 0.0000
    bone.use_connect = True
    bone.parent = arm.edit_bones[bones['Bone.002']]
    bones['Bone.001'] = bone.name

    bpy.ops.object.mode_set(mode='OBJECT')
    pbone = obj.pose.bones[bones['Bone']]
    pbone.rigify_type = 'spines.basic_tail'
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['Bone.002']]
    pbone.rigify_type = ''
    pbone.lock_location = (False, False, False)
    pbone.lock_rotation = (False, False, False)
    pbone.lock_rotation_w = False
    pbone.lock_scale = (False, False, False)
    pbone.rotation_mode = 'QUATERNION'
    pbone = obj.pose.bones[bones['Bone.001']]
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
