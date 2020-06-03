#!/usr/bin/python3
# coding=utf-8

# -------------------------------------------------------------------------------
# This file is part of Phobos, a Blender Add-On to edit robot models.
# Copyright (C) 2020 University of Bremen & DFKI GmbH Robotics Innovation Center
#
# You should have received a copy of the 3-Clause BSD License in the LICENSE file.
# If not, see <https://opensource.org/licenses/BSD-3-Clause>.
# -------------------------------------------------------------------------------

import os
import bpy
import mathutils
import math
import phobos.defs as defs
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
import phobos.utils.selection as sUtils
import phobos.utils.editing as eUtils
import phobos.io.meshes.meshes as meshes
from phobos.model.materials import assignMaterial
from phobos.phoboslog import log
from phobos.utils.validation import validate


def getLargestDimension(geometry):
    """

    Args:
      geometry: 

    Returns:

    """
    # DOCU add some docstring
    if geometry['type'] == 'box':
        return max(geometry['size'])
    if geometry['type'] == 'cylinder':
        return max((geometry['radius'], geometry['length']))
    if geometry['type'] == 'sphere':
        return geometry['radius']
    if geometry['type'] == 'mesh':
        # scale would make no sense here for an absolute measure
        return max(geometry['size']) if 'size' in geometry else 0.2


@validate('geometry_type')
def deriveGeometry(obj, adjust=False, **kwargs):
    """This function derives the geometry from an object.
    
    The returned dictionary contains this information (depending on the geometry type):
        *type*: geometry type of the object
        *size*: dimensions of the object (only for box and mesh)
        *radius*: radius of the object (only for cylinder and sphere)
        *length*: length of the object (only for cylinder)
        *scale*: scale of the object (only for mesh)

    Args:
      obj(bpy_types.Object): object to derive the geometry from
      adjust: (Default value = False)
      **kwargs: 

    Returns:
      : dict -- dictionary representation of the geometry

    """
    geometry = {'type': obj['geometry/type']}
    gtype = obj['geometry/type']

    # enrich the dictionary with sizes, lengths, etc depending on geometry type
    if gtype == 'box':
        geometry['size'] = list(obj.dimensions)

    elif gtype == 'cylinder':
        geometry['radius'] = obj.dimensions[0] / 2
        geometry['length'] = obj.dimensions[2]

    elif gtype == 'sphere':
        geometry['radius'] = obj.dimensions[0] / 2

    elif gtype == 'mesh':
        geometry['filename'] = obj.data.name
        geometry['scale'] = deriveScale(obj)
        # FIXME: is this needed to calculate an approximate inertia
        geometry['size'] = list(obj.dimensions)

    return geometry


def deriveScale(obj):
    """Returns the scale of the specified object.
    
    Object scale is gathered from the matrix_world, as the link scales in Blender might change the
    mesh scale, too.

    Args:
      obj(bpy.types.Object): object to derive the scale of

    Returns:
      list: three scale floats (x, y, z) combined from all parents and the object itself

    """
    return list(obj.matrix_world.to_scale())


def createGeometry(viscol, geomsrc, linkobj, matrix, parent_name, bone_name ):
    """Creates Blender object for visual or collision objects.
    
    If the creation fails, nothing is returned.
    
    These entries in the dictionary are mandatory:
    
    |   **geometry**:
    |       **type**: type of geometry (mesh, box, cylinder, sphere)
    
    Depending on the geometry type other values are required: `size`, `radius`, `length`
    
    These entries are optional:
    
    |   **geometry**:
    |       **scale**: scale for the new geometry
    |   **material**: material name to assign to the visual
    |   **pose**: specifies the placement of the new object relative to the optional linkobj
    |       **translation**: position vector for the new object
    |       **rotation_euler**: rotation for the new object
    
    Furthermore any generic properties, prepended by a ``$`` will be added as custom properties to
    the visual/collision object. E.g. ``$test/etc`` would be put to visual/test/etc for a visual
    object. However, these properties are extracted only in the first layer of hierarchy.

    Args:
      viscol(dict): visual/collision model dictionary representation
      geomsrc(str): phobostype of the new object
      linkobj(bpy.types.Object, optional): link object to attach the visual/collision object to
    (Default value = None)

    Returns:
      bpy.types.Object or None: the new geometry object or nothing

    """
    if 'geometry' not in viscol or viscol['geometry'] is {}:
        log("Could not create {}. Geometry information not defined!".format(geomsrc), 'ERROR')
        return None

    bpy.ops.object.select_all(action='DESELECT')
    geom = viscol['geometry']

    # create the Blender object
    if geom['type'] == 'mesh':
        bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
        meshname = "".join(os.path.basename(geom["filename"]).split(".")[:-1])
        if not os.path.isfile(geom['filename']):
            log(
                "This path "
                + geom['filename']
                + " is no file. Object "
                + viscol['name']
                + " will have empty mesh!",
                'ERROR',
            )
            bpy.ops.object.add(type='MESH')
            newgeom = bpy.context.active_object
            nUtils.safelyName(newgeom, viscol['name'], phobostype=geomsrc)
        else:
            if meshname in bpy.data.meshes:
                log('Assigning copy of existing mesh ' + meshname + ' to ' + viscol['name'], 'INFO')
                bpy.ops.object.add(type='MESH')
                newgeom = bpy.context.object
                newgeom.data = bpy.data.meshes[meshname]
            else:
                log("Importing mesh for {0} element: '{1}".format(geomsrc, viscol['name']), 'INFO')
                filetype = geom['filename'].split('.')[-1].lower()
                newgeom = meshes.importMesh(geom['filename'], filetype)
                # bpy.data.meshes[newgeom].name = meshname
                if not newgeom:
                    log('Failed to import mesh file ' + geom['filename'], 'ERROR')
                    return
    else:
        if geom['type'] == 'box':
            dimensions = geom['size']
        elif geom['type'] == 'cylinder':
            dimensions = (geom['radius'], geom['length'])
        elif geom['type'] == 'sphere':
            dimensions = geom['radius']
        # TODO add support for heightmap, image, plane and polyline geometries (see sdf!)
        else:
            log(
                "Unknown geometry type of "
                + geomsrc
                + viscol['name']
                + '. Placing empty coordinate system.',
                "ERROR",
            )
            bpy.ops.object.empty_add(type='PLAIN_AXES', radius=0.2)
            obj = bpy.context.object
            obj.phobostype = geomsrc
            nUtils.safelyName(bpy.context.active_object, viscol['name'], phobostype=geomsrc)
            return None
        log("Creating primtive for {0}: {1}".format(geomsrc, viscol['name']), 'INFO')
        newgeom = bUtils.createPrimitive(
            viscol['name'], geom['type'], dimensions, phobostype=geomsrc
        )
        newgeom.select = True
        bpy.ops.object.transform_apply(scale=True)

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geom['type']
    if geomsrc == 'visual':
        if 'material' in viscol:
            assignMaterial(newgeom, viscol['material'])
        else:
            log('No material for visual {}.'.format(viscol['name']), 'WARNING')

    # write generic custom properties
    for prop in viscol:
        if prop.startswith('$'):
            for tag in viscol[prop]:
                newgeom[prop[1:] + '/' + tag] = viscol[prop][tag]

    # make sure the name of the mesh fits the name of the bone
    name = viscol['name'].replace('visual_0_', '')
    name = name.replace('.000', '') 
    nUtils.safelyName(newgeom, name)
    newgeom[geomsrc + '/name'] = name
    newgeom.phobostype = geomsrc

    # place geometric object relative to its parent link
    if linkobj:
        # if 'pose' in viscol:
        #     log("Setting transformation of element: " + viscol['name'], 'DEBUG')
        #     #location = mathutils.Matrix.Translation(viscol['pose']['translation'])
        #     #rotation = (
        #         mathutils.Euler(tuple(viscol['pose']['rotation_euler']), 'XYZ').to_matrix().to_4x4()
        #     #)
        # else:
        #     log("No pose in element: " + viscol['name'], 'DEBUG')
        #     location = mathutils.Matrix.Identity(4)
        #     rotation = mathutils.Matrix.Identity(4)
        
        #eUtils.parentObjectsTo(newgeom, linkobj)
        #bpy.ops.object.parent_set(type='BONE')
        #newgeom.parent_set(type='BONE'

        # "one finger is flipped"-issue fix: if it's the problem finger, flip 180°
        name_list = ['r_gripper_r_finger_link', 
                    'r_gripper_r_finger_tip_link', 
                    'l_gripper_r_finger_link', 
                    'l_gripper_finger_tip_link']

        #location = mathutils.Matrix.Translation(viscol['pose']['translation'])
        #rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
        #mathutils.Euler((euler_rotation[0], euler_rotation[1], euler_rotation[2]), 'XYZ').to_matrix() 
        rot_fix = mathutils.Euler((0.0, 0.0, math.pi / 2), 'XYZ').to_matrix().to_4x4()

        if name == 'base_footprint':
            # this is the inital case
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
            matrix = location * rotation
            #newgeom.matrix_local = matrix 
        else:
            # compute location and rotation relative to parent
            parent = bpy.data.objects[parent_name]
            #parent_location = parent.location
            #mat_parent_loc = parent_location.to_matrix().to_4x4()

            #parent_rotation = parent.rotation_euler
            #mat_parent_rot = parent_rotation.to_matrix().to_4x4()

            # matrix_parent = mat_parent_loc * mat_parent_rot
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
            final_matrix = location * rotation
            #newgeom.matrix_local = final_matrix


#        if name in name_list:
#            mat_rot = mathutils.Euler((math.pi, 0.0, 0.0), 'XYZ').to_matrix()
#            newgeom.matrix_local = location * rotation
#        else:
#            newgeom.matrix_local =  location * rotation

        #bpy.data.objects[name].location = viscol['pose']['translation']
        #bpy.data.objects[name].rotation_euler = viscol['pose']['rotation_euler']
 
        armature = bpy.data.objects['armature_object']
        newgeom.parent = armature
        newgeom.parent_type = 'BONE'
        newgeom.parent_bone = bone_name #TODO this needs fixing
        
        mat_rot = mathutils.Euler((- math.pi / 2, 0.0, 0.0), 'XYZ').to_matrix().to_4x4()
        newgeom.matrix_world = matrix #* mat_rot
        bpy.ops.object.transform_apply(location = True, scale = True, rotation = True)

        log("matrix used for visual object used: '{}'".format(matrix), 'INFO')
        log("parent_bone for visual is: '{}'".format(bone_name), 'INFO')
        # # go the vertex route: aka. create a vertex group with the same name as the bone name
        # NOTE: Vertex groups are only needed for deformation of bones, which we do not want here. 
        #vertex_group = newgeom.vertex_groups.new(name)
        #vertices = []
        #for vertex in newgeom.data.vertices:
        #    vertices.append(vertex.index)
        #vertex_group.add(vertices, 1.0, 'ADD')

        # Add armature modifiert
       # bpy.ops.object.modifier_add(type='ARMATURE')
       # bpy.context.object.modifiers["Armature"].object = bpy.data.objects["armature_object"]
       # bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Armature")
        
        
        # index_list = [0]*len(newgeom.data.vertices)
        # newgeom.data.vertices.foreach_get('index', index_list)
        # newgeom.vertex_groups[bone_name].add(index_list, 1, 'REPLACE')

        

    # scale imported object
    if 'scale' in geom:
        newgeom.scale = geom['scale']

    # make object smooth
    eUtils.smoothen_surface(newgeom) # TODO comment this back in? 

    return newgeom
