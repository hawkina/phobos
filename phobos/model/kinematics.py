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


def addPR2KinematicsConstraints():
    # create Control Bones
    createControlBones('r_wrist_flex_link', 'r_arm_control')
    createControlBones('l_wrist_flex_link', 'l_arm_control')

    #createControlBones('r_gripper_tool_frame', 'r_gripper_control')
    #createControlBones('l_gripper_tool_frame', 'l_gripper_control')

    lockAxisForKinematics('r_shoulder_lift_link', 1, 1, 0)
    lockAxisForKinematics('l_shoulder_lift_link', 1, 1, 0)

    lockAxisForKinematics('r_wrist_flex_link', 0, 1, 1)
    lockAxisForKinematics('l_wrist_flex_link', 0, 1, 1)

    lockAxisForKinematics('r_elbow_flex_link', 0, 1, 1)
    lockAxisForKinematics('l_elbow_flex_link', 0, 1, 1)

    addInverseKinematics('r_wrist_flex_link', 'r_arm_control', 3)
    addInverseKinematics('l_force_torque_adapter_link', 'l_arm_control', 4) # this is iai pr2 specific

    addInverseKinematics('l_gripper_l_finger_tip_link', 'l_arm_control', 3)
    addInverseKinematics('l_gripper_r_finger_tip_link', 'l_arm_control', 3)
    addInverseKinematics('r_gripper_l_finger_tip_link', 'r_arm_control', 3)
    addInverseKinematics('r_gripper_r_finger_tip_link', 'r_arm_control', 3)

    # lock axis for left gripper
    lockAxisForKinematics('l_gripper_r_finger_tip_link', 1, 1, 0)
    lockAxisForKinematics('l_gripper_l_finger_tip_link', 1, 1, 0)

    lockAxisForKinematics('l_gripper_r_finger_link', 1, 1, 0)
    lockAxisForKinematics('l_gripper_l_finger_link', 1, 1, 0)

    lockAxisForKinematics('l_force_torque_adapter_link', 1, 1, 1)

    # lock axis for right gripper
    lockAxisForKinematics('r_gripper_r_finger_tip_link', 1, 1, 0)
    lockAxisForKinematics('r_gripper_l_finger_tip_link', 1, 1, 0)

    lockAxisForKinematics('r_gripper_l_finger_link', 1, 1, 0)
    lockAxisForKinematics('r_gripper_r_finger_link', 1, 1, 0)
