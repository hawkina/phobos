#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

"""
Contains all functions to model links within Blender.
"""

import bpy
import mathutils
import re
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
import phobos.model.inertia as inertia
import phobos.model.geometries as geometrymodel
from phobos.phoboslog import log
import operator
import numpy


def getGeometricElements(link):
    """Returns all geometric elements of a link, i.e. 'visual' and 'collision' objects.

    Args:
      link(dict): definition of link
    Returns(list): lists of visual and collision object definitions

    Returns:

    """
    visuals = []
    collisions = []
    if 'visual' in link:
        visuals = [link['visual'][v] for v in link['visual']]
    if 'collision' in link:
        collisions = [link['collision'][v] for v in link['collision']]
    return visuals, collisions

def generateMatrix(model, link, pose, armature):
    "Gets the model object and a link, calculates the matrix and returns it"
    #log("generate matrix called with '{}' ".format(link['name']), 'INFO')
    if pose == 0.0:
        pose = armature.data.edit_bones[link['name']].parent.tail

    #log("pose of parent '{}' ".format(pose), 'INFO')
    euler_rotation = model['links'][link['name']]['pose']['rotation_euler']

    mat_location = mathutils.Matrix.Translation((pose[0], pose[1], pose[2]))
    mat_rotation = mathutils.Euler((euler_rotation[0], euler_rotation[1], euler_rotation[2]), 'XYZ').to_matrix()
    mat_final = mat_location.to_4x4() * mat_rotation.to_4x4()

    return mat_final

def collectParentMatrixes(model, link, pose, mat, armature):
    #log("collect parent matrix called with '{}' ".format(link['name']), 'INFO')
    new_mat = generateMatrix(model, link, pose, armature)
    mat = new_mat *  mat

    try:
        model['links'][link['name']]['parent']
        collectParentMatrixes(model, model['links'][link['name']]['parent'], 0.0, mat, armature)
    except KeyError:
        log("No more parents.", 'INFO')

    return mat
            

# DOCU we should add the parameters, that can be inserted in the dictionary
def createLink(link, model, visited_links, counter):
    """Creates the blender representation of a given link and its parent joint.
    
    The link is added to the link layer.
    
    These entries in the dictionary are mandatory:
        *name*: name for the link
    
    The specified dictionary may contain these entries:
        *matrix*: world matrix for the new link transformation
        *scale*: scale for the new link (single float)
        *visual*: list of visual dictionaries
        *collision*: list of collision dictionaries
        *inertial*: inertial dictionary (an inertial object will be created on the fly)
    
    Furthermore any generic properties, prepended by a `$` will be added as custom properties to the
    link. E.g. $test/etc would be put to link/test/etc. However, these properties are extracted
    only in the first layer of hierarchy.

    Args:
      link(dict): The link you want to create a representation of.

    Returns:
      : bpy_types.Object -- the newly created blender link object.

    """
    log("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    log("Creating link object '{}'...".format(link['name']), 'INFO')
    tiny_bone_length = 0.01
    # create armature/bone
    # ---NEW ---
    # check if link name is base_link
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    bpy.ops.object.select_all(action='DESELECT')
    
    visuals, collisions = getGeometricElements(link)
    newlink = model['links'][link['name']]  

    # case: create the initial bone
    if not bpy.context.blend_data.armatures :
        #log("No armatures found, result was '{}' ".format(bpy.context.blend_data.armatures), 'INFO' )
        #log("Create a new armature", 'INFO')
        #log("Name of root bone: '{}'".format(link['name']), 'INFO' )

        # create empty 
        empty = bpy.data.objects.new("pr2_empty", None)
        bpy.context.scene.objects.link(empty)
        bpy.context.scene.objects.active = empty

        #Create a new armature and the coresponding Armature Object: 
        armature = bpy.data.armatures.new('robot_armature')
        armature = bpy.data.objects.new('armature', armature)
        bpy.context.scene.objects.link(armature)

        #parent armature to empty
        armature.parent = empty

        # create root bone
        armature = bpy.data.objects['armature']
        bpy.context.scene.objects.active = armature
        bpy.ops.object.mode_set(mode='EDIT', toggle=False) # enable edit mode   

        bone = armature.data.edit_bones.new('base_footprint') #create bone
        bone.head = (0.0, 0.0, 0.0) # assuming this is the first bone 
        pose = model['links'][model['links'][link['name']]['children'][0]]['pose']['translation']
        #log("pose of base_footprint tail'{}' ".format((pose[0], pose[1], pose[2])), 'INFO')
        bone.tail = (0.0, 0.0, 0.001) # without this there are two bones with the same location and same size. Model building dies then.
        # add the pose to the relative_poses list with the name as key

        euler_rotation = model['links'][link['name']]['pose']['rotation_euler'] # get euler

        mat_location = mathutils.Matrix.Translation((0.0, 0.0, 0.001))
        mat_rotation = mathutils.Euler((euler_rotation[0], euler_rotation[1], euler_rotation[2]), 'XYZ').to_matrix() # make rotation matrix
        mat_current = mat_location.to_4x4() * mat_rotation.to_4x4() # combine them into one
        mat_parent = mathutils.Matrix.Translation((0.0, 0.0, 0.0)) #translation
        mat_final = mat_parent * mat_current

        #generate bone
        bone.use_deform = False
    
        bpy.ops.object.mode_set(mode='OBJECT', toggle=True) # exit edit mode
        newlink = bone
        bone_name = bone.name
        parent_name = 'base_footprint'
    #-------------------------------------------------------------------------------------------------------------------------
    else:
        # case: armature and initial bone exists. Create next bone.
        bpy.ops.object.select_all(action='DESELECT')
        armature = bpy.data.objects['armature']
        bpy.context.scene.objects.active = armature
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)

        pose = model['links'][link['name']]['pose']['translation'] #translation of current. should be used for head  
        parent_name = model['links'][model['links'][link['name']]['parent']]['name']
        parent = model['links'][parent_name]
     
        parent_pose = armature.data.edit_bones[parent_name].tail
        #parent_pose = model['links'][parent_name]['pose']['translation']

        # make sure bone length can't be 0
        if not ((pose[0] == 0.0 and pose[1] == 0.0 and pose[2] == 0.0) or parent_pose == pose):
            # case: child_pose is not 0, therefore bone should be created.
            # Generate matrix for Bone: 
            #log("Child Pose was not 0. Create bone.", 'INFO')
            mat_location = mathutils.Matrix.Translation((pose[0], pose[1], pose[2])) # make location matrix
            #log("mat_location: '{}'".format(mat_location), 'WARNING')

            euler_rotation = model['links'][link['name']]['pose']['rotation_euler'] # get euler

            mat_rotation = mathutils.Euler((euler_rotation[0], euler_rotation[1], euler_rotation[2]), 'XYZ').to_matrix() # make rotation matrix
            mat_current = mat_location.to_4x4()  # combine them into one
            # check if parent_pose was also 0.0 - find the last parent which had a pose.
            #log("pre-while parent name'{}'".format(parent_name), 'INFO')
            #log("pre-while parent pose'{}'".format(parent_pose), 'INFO')
            while parent_pose[0] == 0.0 and parent_pose[1] == 0.0 and parent_pose[2] == 0.0 and parent_name != 'base_footprint':
                parent = model['links'][parent_name]['parent']
                #log("in-while parent'{}'".format(parent), 'INFO') 
                parent_name = model['links'][parent]['name']
                parent_pose = model['links'][parent_name]['pose']['translation'] 


            parent_pose = armature.data.edit_bones[parent_name].tail
            parent_bone = armature.data.edit_bones[parent_name]

            #make sure that if previous bone is one of the tiny artificial ones, to subtract that from the tail of new bone
            if parent_bone.length <= tiny_bone_length:
                #translation
                mat_parent = mathutils.Matrix.Translation((parent_pose[0] - tiny_bone_length, parent_pose[1], parent_pose[2])).to_4x4() 
            else:
                mat_parent = mathutils.Matrix.Translation((parent_pose[0], parent_pose[1], parent_pose[2])).to_4x4() 

            mat_final = mat_parent * mat_current * mat_rotation.to_4x4()
            
            #This needs to happen no matter if the previous try-catch was succesfull or not
            #bUtils.toggleLayer(defs.layerTypes['link'], True)
            armature = bpy.data.objects['armature']
            bpy.context.scene.objects.active = armature
            bpy.ops.object.mode_set(mode='EDIT')

            bone = armature.data.edit_bones.new(link['name'])
            bone.parent = armature.data.edit_bones[parent_name]
            bone.matrix = mat_final
            #log("Resulting pose: '{}'".format(armature.data.edit_bones[link['name']].tail), 'WARNING')

             # parent bone
            bone.use_connect = True 
        
            # general properties of bones: 
            bone.use_deform = False
    
            bpy.ops.object.mode_set(mode='OBJECT', toggle=True) # exit edit mode
            newlink = bone
            #log("Bone name: '{}'".format(bone.name), 'INFO')
            #log("\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n", 'INFO')
            # spawn meshes of bones in Armature
            bone_name = bone.name
        else: 
            # if child pose is 0
            # if bone is of size 0, force creation by giving it a minimal lengh


            ## if bone is of size 0, return the matrix of parent bone as location
            ##log("got into else with child_pose: '{}'".format(pose))
            parent_name = model['links'][model['links'][link['name']]['parent']]['name']
            parent_pose = model['links'][parent_name]['pose']['translation']
            
            #log("current link: '{}'".format(link['name']), 'INFO')
            #log("parent name: '{}'".format(parent_name), 'INFO')
            # check if parent_pose was also 0.0
            while parent_pose[0] == 0.0 and parent_pose[1] == 0.0 and parent_pose[2] == 0.0:
                parent = model['links'][parent_name]['parent']
            #    #log("in-while parent'{}'".format(parent), 'INFO') 
                parent_name = model['links'][parent]['name']
                parent_pose = model['links'][parent_name]['pose']['translation']    

            parent_pose = armature.data.edit_bones[parent_name].tail
            mat_final = mathutils.Matrix.Translation((parent_pose[0] + tiny_bone_length, parent_pose[1], parent_pose[2])).to_4x4()
            

            #create new bone 
            bone = armature.data.edit_bones.new(link['name'])
            bone.parent = armature.data.edit_bones[parent_name]
            bone.matrix = mat_final

            # parent bone
            bone.use_connect = True 
        
            # general properties of bones: 
            bone.use_deform = False

            #print bone length for debugging
            #log('bone name: {}'.format(bone.name), 'INFO')
            #log('bone lenght:{} '.format(bone.length), 'INFO')

            #newlink = model['links'][link['name']]
            newlink = bone
            bpy.ops.object.mode_set(mode='OBJECT', toggle=True) 
            #debugging
            bone_name = bone.name
            
       
    bpy.ops.object.transform_apply(scale = False, rotation = True)         
    

    #TODO Hasu: Comment these back in
    #create geometric elements
    #log(
    #    "Creating visual and collision objects for link '{0}':\n{1}".format(
    #        link['name'], '    \n'.join([elem['name'] for elem in visuals + collisions])
    #    ),
    #    'DEBUG',
    #)

    #for vis in visuals:
    try: 
        mesh = geometrymodel.createGeometry(visuals[0], 'visual', newlink) #TODO this is currently buggy
    except IndexError:
        log("Index out of Bounds. No Visual for {}".format(newlink), 'WARNING')

    log("newly created bone: '{}'".format(bone_name), 'INFO')
    log("newly created bone's parent: '{}'".format(parent_name), 'INFO')
    bUtils.update()
    return newlink


def deriveLinkfromObject(obj, scale=0.2, parent_link=True, parent_objects=False, nameformat=''):
    """Derives a link from an object using its name, transformation and parenting.

    Args:
      obj(bpy_types.Object): object to derive a link from
      scale(float, optional): scale factor for bone size (Default value = 0.2)
      parent_link(bool, optional): whether to automate the parenting of the new link or not. (Default value = True)
      parent_objects(bool, optional): whether to parent all the objects to the new link or not (Default value = False)
      nameformat(str, optional): re-formatting template for obj names (Default value = '')

    Returns:
      : newly created link

    """
    log('Deriving link from ' + nUtils.getObjectName(obj), level="INFO")
    try:
        nameparts = [p for p in re.split('[^a-zA-Z]', nUtils.getObjectName(obj)) if p != '']
        linkname = nameformat.format(*nameparts)
    except IndexError:
        log('Invalid name format (indices) for naming: ' + nUtils.getObjectName(obj), 'WARNING')
        linkname = 'link_' + nUtils.getObjectName(obj)
    link = createLink({'scale': scale, 'name': linkname, 'matrix': obj.matrix_world})

    # parent link to object's parent
    if parent_link:
        if obj.parent:
            eUtils.parentObjectsTo(link, obj.parent)
    # parent children of object to link
    if parent_objects:
        children = [obj] + sUtils.getImmediateChildren(obj)
        eUtils.parentObjectsTo(children, link, clear=True)
    return link


def setLinkTransformations(model, parent):
    """Assigns the transformations recursively for a model parent link according to the model.
    
    This needs access to the **object** key of a link entry in the specified model.
    The transformations for each link object are extracted from the specified model and applied to
    the Blender object.

    Args:
      parent(dict): parent link you want to set the children for.
      model(dict): model dictionary containing the **object** key for each link

    Returns:

    """
    #bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
    for chi in parent['children']:
        child = model['links'][chi]

        # apply transform as saved in model
        location = mathutils.Matrix.Translation(child['pose']['translation'])
        rotation = (
            mathutils.Euler(tuple(child['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        )

        log("Transforming link {0}.".format(child['name']), 'DEBUG')
        transform_matrix = location * rotation
        child['object'].matrix_local = transform_matrix

        # traverse the tree
        setLinkTransformations(model, child)
