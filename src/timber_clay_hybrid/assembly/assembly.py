from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from assembly_information_model.assembly import Assembly

from compas.geometry import Frame
from compas.geometry import Vector
from compas.geometry import Transformation
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import distance_point_point
from compas_ghpython.artists import MeshArtist
from compas.datastructures import Network, mesh_offset

import rhinoscriptsyntax as rs

import math

from timber_clay_hybrid.export import Exporter
from .element import TCHElement

class TCHAssembly(Assembly):
    """A data structure for discrete element assemblies for human robot collaboration
    """

    def __init__(self,
                 elements=None,
                 attributes=None,
                 default_element_attribute=None,
                 default_connection_attributes=None):

        super(TCHAssembly, self).__init__()
