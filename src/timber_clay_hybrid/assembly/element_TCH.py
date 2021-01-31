from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from assembly_information_model.assembly import Element

import json

from compas.geometry import Frame
from compas.geometry import Transformation
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import Point
from compas.geometry import Box
from compas.datastructures import Mesh
from compas.datastructures import mesh_transform

from assembly_information_model.assembly.utilities import _deserialize_from_data
from assembly_information_model.assembly.utilities import _serialize_to_data

__all__ = ['Element']


# class TCHElement(Element, dims, lay, global_c, layer_c, z, len_dir, w_dir, grid_pos, vert_support, perpendicular=False, loc="centre")
class TCHElement(Element):
    # Data structure representing a discrete element of an assembly.

    def __init__(self, frame, dims, lay, global_c, layer_c, grid_pos, tool_fram, box, vert_support, perpendicular):
        super(TCHElement, self).__init__(frame)

        self.layer = lay
        self.global_count = global_c
        self.no_in_layer = layer_c

        self.frame = frame
        self.tool_frame = tool_fram
        self.box = box
        self.width = dims[0]
        self.height = dims[1]
        self.length = dims[2]
        self.grid_position = grid_pos
        self.vert_sup = vert_support
        self.perp = perpendicular
        self.centre_point = self.frame[0]
        self.length_vector = self.frame[1]
        self.width_vector = self.frame[2]

        self.glue_givers = []
        self.glue_receivers = []
        self.glue_surfaces = []
        self.glue_paths = []
        self.final_glue_frames = []
        self.assembly_path = []
        self.glue_transformations = []
        self.receiving_neighbours = []
        self.giving_neighbours = []

        self.stack_index = None
        self.stack_pick_frame = None
        self.stack_center_frame = None
