from compas.geometry import Point, Box, Frame, Vector, Plane, scale_vector, normalize_vector, Polygon, Rotation, intersection_line_line
from compas.geometry import Transformation

frame1 = Frame(Point(0,0), Vector(-1,0), Vector(0,1))
frame2 = Frame(Point(10,0), Vector(-1,0), Vector(0,1))

frame3 = Frame(Point(0,0), Vector(0,1), Vector(1,0))

F1 = Transformation.from_change_of_basis(frame1, frame2)

frame4 = frame3.transformed(F1)

print(frame4)
