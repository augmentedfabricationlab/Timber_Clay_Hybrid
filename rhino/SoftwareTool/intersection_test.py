from compas.geometry import Point, Box, Frame, Vector, Plane, scale_vector, normalize_vector, Polygon, Rotation, intersection_line_line


a = Vector(0,5,0)
b = Vector(5,0,0)

print(Vector.angle_signed(a,b,Vector(0,0,1)))
