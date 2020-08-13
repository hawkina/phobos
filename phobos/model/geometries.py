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
import queue
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
        #geometry['scale'] = deriveScale(obj) 
        # FIXME: is this needed to calculate an approximate inertia
        #geometry['size'] = list(obj.dimensions)

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


def createGeometry(viscol, geomsrc, linkobj):
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
        #bpy.context.scene.layers = bUtils.defLayers(defs.layerTypes[geomsrc])
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
        # HASU: used to contain scale=True, removed for testing
        #bpy.ops.object.transform_apply(scale=False)

    # from here it's the same for both meshes and primitives
    newgeom['geometry/type'] = geom['type']
    if geomsrc == 'visual':
        if 'material' in viscol:
            assignMaterial(newgeom, viscol['material'])
        else:
            log('No material for visual {}.'.format(viscol['name']), 'DEBUG')

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

        #location = mathutils.Matrix.Translation(viscol['pose']['translation'])
        #rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
        #mathutils.Euler((euler_rotation[0], euler_rotation[1], euler_rotation[2]), 'XYZ').to_matrix() 
        #rot_fix = mathutils.Euler((0.0, 0.0, math.pi / 2), 'XYZ').to_matrix().to_4x4()

        # set current scale as default. otherwise there will eb scalign issues!
        bpy.ops.object.transform_apply(scale = True) 
        if name == 'base_footprint':
            # this is the inital case
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
            final_matrix = rotation * location
            #newgeom.matrix_local = matrix 
        else:
            # compute location and rotation relative to parent
            #parent = bpy.data.objects[parent_name]
            #parent_location = parent.location
            #mat_parent_loc = parent_location.to_matrix().to_4x4()

            #parent_rotation = parent.rotation_euler
            #mat_parent_rot = parent_rotation.to_matrix().to_4x4()

            # matrix_parent = mat_parent_loc * mat_parent_rot
            location = mathutils.Matrix.Translation(viscol['pose']['translation'])
            rotation = mathutils.Euler(viscol['pose']['rotation_euler'], 'XYZ').to_matrix().to_4x4()
            final_matrix = rotation * location 
            #newgeom.matrix_local = final_matrix

        #bpy.data.objects[name].location = viscol['pose']['translation']
        #bpy.data.objects[name].rotation_euler = viscol['pose']['rotation_euler']
        #! This works!!!
        #armature = bpy.data.objects['armature']
        #newgeom.parent = armature
        #newgeom.parent_type = 'OBJECT'
        #newgeom.parent_bone = bone_name #TODO this needs fixing


        newgeom.matrix_world =  final_matrix
        # HaSu: Scale=true set to false for testing
        bpy.ops.object.transform_apply(location = True, rotation = True)
        #log("matrix used for visual object used: '{}'".format(matrix), 'INFO')
        #log("parent_bone for visual is: '{}'".format(bone_name), 'INFO')
        # # go the vertex route: aka. create a vertex group with the same name as the bone name
        # NOTE: Vertex groups are only needed for deformation of bones, which we do not want here. 
        #vertex_group = newgeom.vertex_groups.new(name)
        #vertices = []
        #for vertex in newgeom.data.vertices:
        #    vertices.append(vertex.index)
        #vertex_group.add(vertices, 1.0, 'ADD')

        # Add armature modifier
       # bpy.ops.object.modifier_add(type='ARMATURE')
       # bpy.context.object.modifiers["Armature"].object = bpy.data.objects["armature"]
       # bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Armature")
        
        
        # index_list = [0]*len(newgeom.data.vertices)
        # newgeom.data.vertices.foreach_get('index', index_list)
        # newgeom.vertex_groups[bone_name].add(index_list, 1, 'REPLACE')    

    # scale imported object
    #if 'scale' in geom:
    #    newgeom.scale = geom['scale']
    #    log("Scale is: '{}'".format(geom['scale']), 'WARNING')

    # make object smooth
    eUtils.smoothen_surface(newgeom) # TODO comment this back in? 
    #parent mesh to empty for now
    bpy.data.objects[name].parent = bpy.data.objects["pr2_empty"]

    # apply scale to mesh since the import apperently fales to do so: 
    #bpy.data.objects[name].scale = (0.1, 0.1, 0.1)

    return name

# lists of exceptions of meshes and bones.
# this is robot and maybe even urdf specific
# TODO make these optional depending on robot model?
# these mostly concern the arms
# "move back" means moving the meshes to the position of the parents bone's head
move_back_meshes = ['r_upper_arm_roll_link', 'r_upper_arm_link', #'r_elbow_flex_link',
                    'r_forearm_roll_link', 'r_forearm_cam_frame', 'r_forearm_cam_optical_frame',
                    'r_forearm_link', #'r_wrist_flex_link', 'r_wrist_roll_link', 
                    'r_gripper_palm_link',
                    'r_shoulder_pan_link', 
                    'r_shoulder_lift_link',
                    'r_elbow_flex_link',
                    # newly added:
                    'r_gripper_motor_accelerometer_link', 'r_gripper_motor_slider_link',
                    'r_gripper_motor_screw_link', #'r_gripper_r_finger_tip_link',
                    'r_gripper_l_finger_tip_frame',
                    'r_gripper_r_finger_link',
                    'r_gripper_l_finger_link',
                    
                    # left arm
                    'l_upper_arm_roll_link', 'l_upper_arm_link', #'l_elbow_flex_link',
                    'l_forearm_roll_link', 'l_forearm_cam_frame', 'l_forearm_cam_optical_frame',
                    'l_forearm_link', #'l_wrist_flex_link', 'l_wrist_roll_link', 
                    'l_gripper_palm_link',
                    'l_shoulder_pan_link', 
                    'l_shoulder_lift_link',
                    'l_elbow_flex_link',
                    # newly added for left arm: 
                    'l_gripper_motor_accelerometer_link', 'l_gripper_motor_slider_link',
                    'l_gripper_motor_screw_link', #'l_gripper_r_finger_tip_link',
                    'l_gripper_l_finger_tip_frame',
                    # specific to the left arm due to the IAI force-torque sensor:
                    'l_force_torque_adapter_link', #'l_force_torque_link', 
                    'l_gripper_palm_link',
                    'l_gripper_motor_accelerometer_link',
                    'l_gripper_l_finger_link',
                    'l_gripper_r_finger_link'
                    ]

# enforce binding to child bone instead of same name bone
enforce_parent = ['r_shoulder_pan_link', 
                  'r_shoulder_lift_link',
                  'r_elbow_flex_link',
                  'l_shoulder_pan_link', 
                  'l_shoulder_lift_link',
                  'l_elbow_flex_link',
                  'r_gripper_r_finger_link',
                  'r_gripper_l_finger_link',
                  'l_gripper_r_finger_link',
                  'l_gripper_l_finger_link'
                  ]

parenting_exceptions = ['r_gripper_r_finger_tip_link', 'r_gripper_l_finger_tip_link',
                        'l_gripper_r_finger_tip_link', 'l_gripper_l_finger_tip_link']

def moveAllMeshes(model, visited_meshes, current, unvisited_meshes):
    log("-----------------------------------------------", 'DEBUG')
    log("current: '{}'".format(current), 'DEBUG')
    bpy.ops.object.mode_set(mode='OBJECT') # go back into object mode
    bpy.ops.object.select_all(action='DESELECT')

    # Start with base_footprint
    #new_link = model['links'][current] # initial mesh should be bf
    name = current
    log("Mesh to move: '{}’".format(name), 'DEBUG')

    empty = bpy.data.objects['pr2_empty'] 
    bpy.context.scene.objects.active = empty

    if name not in visited_meshes:
        # access mesh
        try:
            mesh = bpy.data.objects[name] #the actuall mesh     
            log("accessing mesh was succesfull.", 'DEBUG')
            # find the bone to parent the mesh to
            # access armature and switch into EDIT mode
            armature = bpy.data.objects['armature']
            bpy.ops.object.select_all(action='DESELECT')
            armature.select = True
            bpy.context.scene.objects.active = armature
            bpy.ops.object.mode_set(mode='EDIT')

            # find the bone to parent the object to.
            parent_bone = ''
            try:
                log("Try to find bone with same name...", 'DEBUG')
                # check if a bone with the same name as the mesh exists
                parent_bone = armature.data.edit_bones[name]
                parent_bone_name = name
            except KeyError:
                log("Exception. check if link has child?", 'DEBUG')
                # if not, check if a child of it exists as bone
                children = model['links'][name]['children']
                children_of_children = []
                depth = 2 # amount of layers to go through 
                q = queue.Queue()
                q.put(name)
                
                while not q.empty:
                    log("in while... queue: '{}'".format(queue), 'DEBUG')
                    children = model['links'][q.get()]['children']
                    # add children to temp list
                    for child in children:
                        children_of_children.append(child)
                        if depth > 0:
                            q.put(child)
                    depth = depth - 1
                    log("depth level: '{}', current list: '{}'".format(depth, queue), 'ERROR')

                log("into for loop we go!", 'INFO')
                for child in children_of_children: # make sure nill entries are removed from the list
                    if child != []:
                        children.append(child)

                log("list of children to go though: '{}'".format(children), 'DEBUG')

                if children != []:
                    for child in children:
                        try:
                            log("try to set the child '{}' as bone-parent".format(child), 'DEBUG')
                            parent_bone_name = child
                            log("Select a new child bone: '{}’".format(parent_bone_name), 'DEBUG')
                            parent_bone = armature.data.edit_bones[parent_bone_name]
                            break
                        except:
                            log("No bone with this name '{}' found. try next child.".format(parent_bone_name), 'WARNING')
                else:
                    log("No children found. Probably leaf. Try parents instead.", 'DEBUG')
                    
                counter = 2
                potential_parent = name
                while counter <= 2 and parent_bone == '':
                    # try to bind to parent instead.
                    log("parenting to child failed. try to parent to parent", 'DEBUG')
                    try:
                        parent_bone_name = model['links'][potential_parent]['parent']
                        parent_bone = armature.data.edit_bones[parent_bone_name]
                        log("found parent bone with name: '{}'".format(parent_bone_name), 'DEBUG')
                        break
                    except KeyError:
                        log("parenting to parent '{}' failed.".format(parent_bone_name), 'WARNING')
                        counter = counter -1
                        #potential_parent = model['links'][potential_parent]['parent']
                        potential_parent = model['links'][model['links'][potential_parent]['parent']]['name']
                        log("counter: '{}'".format(counter), 'DEBUG')
                        log("new potential parent: '{}'".format(potential_parent), 'DEBUG')
                        #raise
                    log("try again with parent of parent", 'DEBUG')

            # enforce setting child_link as parent_bone for some meshes
            if name in enforce_parent:
                # find usable child
                child = model['links'][name]['children']
                child_search = True
                while child_search:
                    child = child[0]
                    try:
                        armature.data.bones[child]
                        child_search = False
                        parent_bone_name = child
                    except KeyError:
                        log("child with name '{}' does not exist. try next.".format(child), 'DEBUG')
                        child = model['links'][child]['children']
            
            if name in parenting_exceptions:
                parent_bone_name = parent_bone_name + '_tip'


            log("parent_bone name: '{}'".format(parent_bone), 'DEBUG')           
            # assume the above worked, and we now have a parent bone
            armature.data.edit_bones.active = parent_bone

            bpy.ops.object.mode_set(mode='OBJECT') # go back into object mode
            bpy.ops.object.select_all(action='DESELECT')
            mesh.select = True
            armature.select = True
            bpy.context.scene.objects.active = armature

            

            #    parent_bone_name = model['links'][model['links'][parent_bone_name]['parent']]['name']
            #    log("Enforced parent_bone to be not the same as name, but the parent of parent.", 'INFO')

            mesh.parent = armature
            mesh.parent_type = 'BONE'
            mesh.parent_bone = parent_bone_name
            log("Parent is set to '{}'".format(parent_bone_name), 'DEBUG')
            # parenting was successfull, so put the mesh into the list of visited meshes
            visited_meshes[name] = mesh

            # correct position of meshes where necessary. 
            
            # this is relevant mostly for the meshes in the arms:
            # take the position of the bone's head instead of tail
            if name in move_back_meshes:
                loc_parent_bone_head = armature.data.bones[parent_bone_name].head_local
                log("location of parent's bone head: '{}'".format(loc_parent_bone_head), 'DEBUG')
                mat_loc = mathutils.Matrix.Translation((loc_parent_bone_head[0], loc_parent_bone_head[1], loc_parent_bone_head[2])).to_4x4()
                mesh.matrix_world = mat_loc
            else:
                loc_parent_bone_tail = armature.data.bones[parent_bone_name].tail_local
                log("location of parent's bone tail: '{}'".format(loc_parent_bone_tail), 'DEBUG')
                mat_loc = mathutils.Matrix.Translation((loc_parent_bone_tail[0], loc_parent_bone_tail[1], loc_parent_bone_tail[2])).to_4x4()
                mesh.matrix_world = mat_loc

            # add armature modifier to mesh (important for export later)
            log("adding armature modifier to: '{}'".format(mesh), 'INFO')
            bpy.context.scene.objects.active = mesh
            # make sure it is only applied to a mesh object
            # This is very important for the unreal import!!!
            if mesh.type == 'MESH':  
                # add vertex groups
                vertex_group = mesh.vertex_groups.new(name)
                vertices = []
                for vertex in mesh.data.vertices:
                    vertices.append(vertex.index)
                vertex_group.add(vertices, 1.0, 'ADD')

                # add armature modifier
                bpy.ops.object.modifier_add(type='ARMATURE')
                bpy.context.object.modifiers["Armature"].object = bpy.data.objects["armature"]              
                bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Armature")
         
                #index_list = [0]*len(mesh.data.vertices)
                #mesh.data.vertices.foreach_get('index', index_list)
                #mesh.vertex_groups[parent_bone_name].add(index_list, 1, 'REPLACE')  

                
                        

        except:
            log("In moveAllMeshes: No mesh for '{}' exists. Maybe it was already processed?.".format(name), 'WARNING')
            unvisited_meshes.append(name)
            raise
    else:
        log("Mesh is already in visited_meshes list with the name '{}'".format(name), 'ERROR')

    # after everything is done, add this link to visited links
    log("all objects of type mesh: '{}'".format(bpy.data.meshes), 'DEBUG')
    log("all objects in visited meshes: '{}'".format(len(visited_meshes)), 'DEBUG')

    
    

    
        
        

