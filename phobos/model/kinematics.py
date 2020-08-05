import bpy
from phobos.phoboslog import log

def addInverseKinematics(root, target, chain_length):
# this refers to the IK Constrait added onto bones
# root = root of the chain. String.
# target = target of the chain. Which bone should be followed? String of bone name
    armature = bpy.data.objects['armature_object']
    bpy.context.scene.objects.active = armature
    
    bone = armature.pose.bones[root]

    constraint = bone.constraints.new('IK')
    constraint.target = armature
    constraint.subtarget = target
    constraint.chain_count = chain_length

def addCopyRotationConstraint(source, target):
    # adds the copy rotation constraint to a given source bone
    # source: the bone we want to add the constraint to
    # target: the bone we want to copy the rotation from

    armature = bpy.data.objects['armature_object']
    bpy.context.scene.objects.active = armature
    bone = armature.pose.bones[source]

    constraint = bone.constraints.new('COPY_ROTATION')
    constraint.target = armature
    constraint.subtarget = target
    constraint.owner_space = 'LOCAL_WITH_PARENT' 
    constraint.target_space = 'LOCAL_WITH_PARENT'

    constraint.use_x = True
    constraint.use_y = True
    constraint.use_z = True
    constraint.invert_z = True

def lockAxisForKinematics(link_name, x, y, z):
    # this refers to the constraints in the IK tab of each object
    # E.g. use for the both shoulders to prevent them from lifting awkwardly
    armature = bpy.data.objects['armature_object']
    bpy.context.scene.objects.active = armature

    if x:
        armature.pose.bones[link_name].lock_ik_x = True
    if y:
        armature.pose.bones[link_name].lock_ik_y = True
    if z:
        armature.pose.bones[link_name].lock_ik_z = True

def createControlBones(origin, name):
    # origin refers to the origin bone we use to create an offset
    # name is the name of the control bone
    armature = bpy.data.objects['armature_object']
    bpy.context.scene.objects.active = armature

    origin_tail = armature.data.bones[origin].tail_local
    log("tail: '{}'".format(origin_tail), 'ERROR')
    offset = 0.5
    length = 0.3
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)

    bone = armature.data.edit_bones.new(name)
    bone.use_connect = False 

    bone.head = (origin_tail[0] + offset, origin_tail[1], origin_tail[2])
    bone.tail = (origin_tail[0] + offset + length, origin_tail[1], origin_tail[2])

    bpy.ops.object.mode_set(mode='OBJECT', toggle=True) 

def boneModificationsPR2(name, offset_x, offset_y):
    # these are some bone modifications which are specific to the  PR2 and it'S grippers
    # add additional bone to the gripper fingers

    armature = bpy.data.objects['armature_object']
    bpy.context.scene.objects.active = armature
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    bpy.ops.armature.select_all(action='DESELECT')
    # create new bone by subdividing
    bone = armature.data.edit_bones[name]
    armature.data.edit_bones[name].select = True
    bpy.ops.armature.subdivide()

    bpy.ops.object.mode_set(mode='OBJECT', toggle=True)
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)

    # rename bones
    bone = armature.data.edit_bones[name + '.001']
    bone.name = name + '_tip'

    # set position of bone
    head = bone.head
    bone.head = (head[0] + offset_x, head[1] + offset_y, head[2])

def addPR2KinematicsConstraints():
    # create Control Bones
    createControlBones('r_wrist_flex_link', 'r_arm_control')
    createControlBones('l_wrist_flex_link', 'l_arm_control')

    # everything ARM related

    lockAxisForKinematics('r_shoulder_lift_link', 1, 1, 0)
    lockAxisForKinematics('l_shoulder_lift_link', 1, 1, 0)

    lockAxisForKinematics('r_wrist_flex_link', 0, 1, 1)
    lockAxisForKinematics('l_wrist_flex_link', 0, 1, 1)

    lockAxisForKinematics('r_elbow_flex_link', 0, 1, 1)
    lockAxisForKinematics('l_elbow_flex_link', 0, 1, 1)

    addInverseKinematics('r_wrist_flex_link', 'r_arm_control', 3)
    addInverseKinematics('l_force_torque_adapter_link', 'l_arm_control', 4) # this is iai pr2 specific

    # add gripper constraints
    addCopyRotationConstraint('r_gripper_l_finger_tip_link', 'r_gripper_r_finger_tip_link')
    addCopyRotationConstraint('l_gripper_l_finger_tip_link', 'l_gripper_r_finger_tip_link')

