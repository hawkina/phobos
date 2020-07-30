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
Contains the functions for the mesh entity.
"""

import os
import bpy
import phobos.utils.naming as nUtils
import phobos.utils.blender as bUtils
from phobos.phoboslog import log


def exportMesh(obj, path, meshtype):
    """

    Args:
      obj: 
      path: 
      meshtype: 

    Returns:

    """
    import phobos.utils.io as ioUtils

    # DOCU add some docstring
    objname = nUtils.getObjectName(obj)
    tmpobjname = obj.name
    # OPT: surely no one will ever name an object like so, better solution?
    obj.name = 'tmp_export_666'
    tmpobject = bUtils.createPrimitive(objname, 'box', (1.0, 1.0, 1.0))
    # copy the mesh here
    tmpobject.data = obj.data
    outpath = os.path.join(path, obj.data.name + "." + meshtype)
    if meshtype == 'obj':
        axis_forward = bpy.context.scene.phobosexportsettings.obj_axis_forward
        axis_up = bpy.context.scene.phobosexportsettings.obj_axis_up
        bpy.ops.export_scene.obj(
            filepath=outpath,
            use_selection=True,
            use_normals=True,
            use_materials=False,
            use_mesh_modifiers=True,
            axis_forward=axis_forward,
            axis_up=axis_up,
        )
    elif meshtype == 'stl':
        bpy.ops.export_mesh.stl(filepath=outpath, use_selection=True, use_mesh_modifiers=True)
    elif meshtype == 'dae':
        bpy.ops.wm.collada_export(filepath=outpath, selected=True)
    bpy.ops.object.select_all(action='DESELECT')
    tmpobject.select = True
    bpy.ops.object.delete()
    obj.name = tmpobjname


def importMesh(filepath, meshtype):
    """

    Args:
      filepath: 
      meshtype: 

    Returns:

    """
    #log("Meshtype: '{}'".format(meshtype), 'WARNING')
    # DOCU add some docstring
    # tag all objects
    for obj in bpy.data.objects:
        obj['phobosTag'] = True

    # import mesh
    try:
        mesh_type_dict[meshtype]['import'](filepath)
    except KeyError:
        log('Unknown mesh type: ' + meshtype, 'ERROR')

    bl = None
    if 'base_link' in bpy.data.objects:
            bl = bpy.data.objects['base_link']
    if bl:
        log("in importMesh after import base link scale: '{}'".format(bl.scale), 'ERROR')

    # find the newly imported obj
    newgeom = None
    for obj in bpy.data.objects:
        if 'phobosTag' not in obj:
            newgeom = obj

    # with obj file import, blender only turns the object, not the vertices,
    # leaving a rotation in the matrix_basis, which we here get rid of
    if meshtype == 'obj':
        bpy.ops.object.select_all(action='DESELECT')
        newgeom.select = True
        bpy.ops.object.transform_apply(rotation=True, scale=False)

    # clean the tag
    for obj in bpy.data.objects:
        if 'phobosTag' in obj:
            del obj['phobosTag']

    return newgeom


def importObj(filepath):
    """

    Args:
      filepath: 

    Returns:

    """
    # DOCU add some docstring
    log("Importing OBJ", 'ERROR')
    bpy.ops.import_scene.obj(filepath=filepath)


def importStl(filepath):
    """

    Args:
      filepath: 

    Returns:

    """
    # DOCU add some docstring
    #log("Importing STL", 'ERROR')
    # maybe add , use_scene_unit=False ?
    bpy.ops.import_mesh.stl(filepath=filepath, global_scale=0.001)


def importDae(filepath):
    """

    Args:
      filepath:

    Returns:

    """
    # DOCU add some docstring
    #log("importing DAE", 'ERROR')
    bpy.ops.wm.collada_import(filepath=filepath, import_units=True)
    mesh = bpy.context.active_object
    #mesh.scale = (0.001, 0.001, 0.001)
    log("Scale after import: '{}'".format(mesh.scale), 'ERROR')


def exportObj(obj, path):
    """This function exports a specific object to a chosen path as an .obj

    Args:
      path(str): The path you want the object export to. *without the filename!*
      obj(types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'obj')


def exportStl(obj, path):
    """This function exports a specific object to a chosen path as a .stl

    Args:
      path(str): The path you want the object exported to. *without filename!*
      obj(bpy.types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'stl')


def exportDae(obj, path):
    """This function exports a specific object to a chosen path as a .dae

    Args:
      path(str): The path you want the object exported to. *without filename!*
      obj(bpy.types.Object): The blender object you want to export.

    Returns:

    """
    exportMesh(obj, path, 'dae')


# registering mesh types with Phobos
mesh_type_dict = {
    'obj': {'export': exportObj, 'import': importObj, 'extensions': ('obj',)},
    'stl': {'export': exportStl, 'import': importStl, 'extensions': ('stl',)},
    'dae': {'export': exportDae, 'import': importDae, 'extensions': ('dae',)},
}
