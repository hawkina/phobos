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


# DOCU we should add the parameters, that can be inserted in the dictionary
def createLink(link, model, previous):
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
    log("Creating link object '{}'...".format(link['name']), 'DEBUG', prefix='\n')
    # create armature/bone
    # ---NEW ---
    # check if link name is base_link
    bUtils.toggleLayer(defs.layerTypes['link'], True)
    bpy.ops.object.select_all(action='DESELECT')
    # NOTE: bpy.data.armatures['Armature']
    
    if not bpy.context.blend_data.armatures :
        #bpy.context.scene.objects.link( bpy.data.objects.new( "base_footprint", None )) #create and Link an empty object

        log("No armatures found, result was '{}' ".format(bpy.context.blend_data.armatures), 'INFO' )
        log("Create a new armature", 'INFO')
        log("Name of root bone: '{}'".format(link['name']), 'INFO' )
        bpy.ops.object.armature_add(layers=bUtils.defLayers([defs.layerTypes['link']])) #create armature
        
        #bpy.data.objects['base_footprint'].select  = True # select obj the armature belongs to

        
        armature = bpy.context.blend_data.armatures[0] # select armature
        bpy.context.scene.objects.active = bpy.data.objects['Armature']
        bpy.ops.object.mode_set(mode='EDIT', toggle=False) # enable edit mode      
        bone = armature.edit_bones.new('base_footprint') #create bone
        

        
        bone.head = (0.0, 0.0, 0.0) # assuming this is the first bone 
        #model['links'][link['name']]['pose']['translation']
        # parse the string into a triple that can be read by the tail function
        #vector = model['links'][link['name']]['pose']['translation']
        vector = model['links'][model['links'][link['name']]['children'][0]]['pose']['translation']
        log("pose of base_footprint tail'{}' ".format((vector[0], vector[1], vector[2])), 'INFO')
        bone.tail = (vector[0], vector[1], vector[2])
        bone.parent = armature.edit_bones['Bone'] # not sure if this is necessary

    else:
        #bpy.data.objects['base_footprint'].select = True # select obj the armature belongs to
        bpy.data.objects[0].select = True # select obj the armature belongs to
        armature = bpy.context.blend_data.armatures[0]
        log("Found existing Armature with the name: '{}' ".format(armature.name), 'INFO')

        #log("Is in edit mode? check 1 '{}'".format(armature.is_editmode), 'INFO')
        
        bpy.context.scene.objects.active = bpy.data.objects[previous] #bpy.data.objects['base_footprint']
        bpy.ops.object.mode_set(mode='EDIT', toggle=False)
        #log("Is in edit mode? check 2'{}'".format(armature.is_editmode), 'INFO')
        bone = armature.edit_bones.new(link['name'])

        vector = model['links'][model['links'][link['name']]['parent']]['pose']['translation'] #translation of head of bone (parent)
        bone.head = (vector[0], vector[1], vector[2])
        log("pose of bone head'{}' ".format((vector[0], vector[1], vector[2])), 'INFO')
        
        vector = model['links'][link['name']]['pose']['translation'] #translation of tail of bone (child or origin)
        log("pose of bone tail'{}' ".format((vector[0], vector[1], vector[2])), 'INFO')
        bone.tail = (vector[0], vector[1], vector[2])

        log("Current Bone: '{}'".format(link['name']), 'INFO')
        
        log("Parent Bone: '{}'".format(model['links'][model['links'][link['name']]['parent']]['name']), 'INFO')
        #log("Child Bone: '{}'".format(model['links'][model['links'][link['name']]['child']]['name']), 'INFO')
        bone.parent = armature.edit_bones[model['links'][model['links'][link['name']]['parent']]['name']] # parent  bone
    
    bpy.ops.object.mode_set(mode='OBJECT')
    newlink = bpy.context.active_object
    # -----
    
    # bUtils.toggleLayer(defs.layerTypes['link'], True)
    # bpy.ops.object.select_all(action='DESELECT')
    # bpy.ops.object.armature_add(layers=bUtils.defLayers([defs.layerTypes['link']]))
    # newlink = bpy.context.active_object

    # Move bone when adding at selected objects location
    if 'matrix' in link:
        newlink.matrix_world = link['matrix']

    # give it a proper name
    newlink.phobostype = 'link'
    if link['name'] in bpy.data.objects.keys():
        log('Object with name of new link already exists: ' + link['name'], 'WARNING')
    nUtils.safelyName(newlink, link['name'])

    # set the size of the link
    visuals, collisions = getGeometricElements(link)
    if visuals or collisions:
        scale = max(
            (geometrymodel.getLargestDimension(e['geometry']) for e in visuals + collisions)
        )
    else:
        scale = 0.2

    # use scaling factor provided by user
    if 'scale' in link:
        scale *= link['scale']
    newlink.scale = (scale, scale, scale)
    bpy.ops.object.transform_apply(scale=True)

    # add custom properties
    for prop in link:
        if prop.startswith('$'):
            for tag in link[prop]:
                newlink['link/' + prop[1:] + '/' + tag] = link[prop][tag]

    # create inertial
    if 'inertial' in link:
        inertia.createInertial(link['inertial'], newlink)

    # create geometric elements
    log(
        "Creating visual and collision objects for link '{0}':\n{1}".format(
            link['name'], '    \n'.join([elem['name'] for elem in visuals + collisions])
        ),
        'DEBUG',
    )
    for vis in visuals:
        geometrymodel.createGeometry(vis, 'visual', newlink)
    for col in collisions:
        geometrymodel.createGeometry(col, 'collision', newlink)
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
    bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes['link'])
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
