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


class TCHElement(Element):
    """Data structure representing a discrete element of an assembly.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame of the element.

    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(TCHElement, self).__init__(frame)
        
