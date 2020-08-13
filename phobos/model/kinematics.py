import bpy
from phobos.phoboslog import log

def addInverseKinematics(root, target, chain_length):
# this refers to the IK Constrait added onto bones
# root = root of the chain. String.
# target = target of the chain. Which bone should be followed? String of bone name
    armature = bpy.data.objects['armature']
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

    armature = bpy.data.objects['armature']
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

def addLockRotationConstraint(name, x, y, z):
    armature = bpy.data.objects['armature']
    bpy.context.scene.objects.active = armature
    bone = armature.pose.bones[name]


    constraint = bone.constraints.new('LIMIT_ROTATION')
    constraint.owner_space = 'LOCAL_WITH_PARENT' 
    if x:
        constraint.use_limit_x = True
    if y: 
        constraint.use_limit_y = True
    if z:
        constraint.use_limit_z = True


def lockAxisForKinematics(link_name, x, y, z):
    # this refers to the constraints in the IK tab of each object
    # E.g. use for the both shoulders to prevent them from lifting awkwardly
    armature = bpy.data.objects['armature']
    bpy.context.scene.objects.active = armature
    bone = armature.pose.bones[link_name]

    if x and not bone.use_ik_limit_x:
        bone.lock_ik_x = True
    if y and not bone.use_ik_limit_y:
        bone.lock_ik_y = True
    if z and not bone.use_ik_limit_z:
        bone.lock_ik_z = True

def createControlBones(origin, name):
    # origin refers to the origin bone we use to create an offset
    # name is the name of the control bone
    armature = bpy.data.objects['armature']
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

exceptions = ['r_gripper_tool_frame', 'l_gripper_tool_frame']
def subdivideBone(name, h_offset_x, h_offset_y, new_name = '', tiny_bone = False, t_offset_x = 0, t_offset_y = 0):
    # these are some bone modifications which are specific to the  PR2 and it'S grippers
    # add additional bone to the gripper fingers

    armature = bpy.data.objects['armature']
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
    if new_name == '':
        bone = armature.data.edit_bones[name + '.001']
        bone.name = name + '_tip'
    else:
        bone = armature.data.edit_bones[name].name = new_name
        bone = armature.data.edit_bones[name + '.001']
        bone.name = name

    # set position of bone
    head = bone.head
    bone.head = (head[0] + h_offset_x, head[1] + h_offset_y, head[2])

    if tiny_bone: 
        bone = armature.data.edit_bones[new_name]
        tail = bone.head
        bone.tail = (tail[0] + t_offset_x, tail[1] + t_offset_y, tail[2])

    #if name in exceptions:
    #    if name[0] == 'r':
    #        # get pose of 'r_gripper_r_finger_link
    #        orig_tail = bone.tail
    #        pose = armature.data.edit_bones['r_gripper_r_finger_tip_link'].tail
    #        x_diff = pose[0] - orig_tail[0]
    #        bone.tail = (orig_tail[0] + x_diff, orig_tail[1], orig_tail[2])

def reParentBone(source, target):
    # used to prettify the gripper
    # reparent bones to have one parent instead of two
    armature = bpy.data.objects['armature']
    bpy.context.scene.objects.active = armature
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    bpy.ops.armature.select_all(action='DESELECT')

    # split source bone of armature
    bone = armature.data.edit_bones[source]
    armature.data.edit_bones[source].select = True
    bpy.ops.armature.split()

    # bind bone to new parent
    bone.parent = armature.data.edit_bones[target]
    bone.use_connect = True

def deleteBone(name):
    # delete the selected bone
    armature = bpy.data.objects['armature']
    bpy.context.scene.objects.active = armature
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    bpy.ops.armature.select_all(action='DESELECT')

    armature.data.edit_bones[name].select = True
    bpy.ops.armature.delete()



def boneModificationsPR2():
    # fix the gripper tips issues
    subdivideBone('r_gripper_r_finger_tip_link', 0.03, -0.015)
    subdivideBone('r_gripper_l_finger_tip_link', 0.03, 0.015)

    subdivideBone('l_gripper_r_finger_tip_link', 0.03, -0.015) # option: 0.015, - 0.035
    subdivideBone('l_gripper_l_finger_tip_link', 0.03, 0.015)

    # fix the gripper palm link issue
    subdivideBone('r_gripper_tool_frame', 0.00, 0.00, 'r_gripper_palm_frame', True, 0.01)
    reParentBone('r_gripper_r_finger_link', 'r_gripper_palm_frame')
    reParentBone('r_gripper_l_finger_link', 'r_gripper_palm_frame')

    reParentBone('r_gripper_r_finger_tip_link', 'r_gripper_r_finger_link')
    reParentBone('r_gripper_l_finger_tip_link', 'r_gripper_l_finger_link')

    reParentBone('r_gripper_r_finger_tip_link_tip', 'r_gripper_r_finger_tip_link')
    reParentBone('r_gripper_l_finger_tip_link_tip', 'r_gripper_l_finger_tip_link')

    reParentBone('r_gripper_motor_slider_link', 'r_gripper_palm_frame')
    reParentBone('r_gripper_tool_frame', 'r_gripper_palm_frame')

    #deleteBone('r_gripper_r_finger_link')
    #deleteBone('r_gripper_l_finger_link')

    #delete unecessary bones
    deleteBone('r_gripper_led_frame')
    deleteBone('l_gripper_led_frame')

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

    # lock some axis for gripper so that figers follow root bone of gripper
    addLockRotationConstraint('r_gripper_r_finger_link', 1, 1, 1)
    addLockRotationConstraint('r_gripper_l_finger_link', 1, 1, 1)
    addLockRotationConstraint('r_gripper_tool_frame', 1, 1, 1)
    addLockRotationConstraint('r_gripper_motor_slider_link', 1, 1, 1)

