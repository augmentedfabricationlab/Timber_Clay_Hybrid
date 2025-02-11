import math
import copy

from compas.geometry import Point, Box, Frame, Vector, scale_vector
from compas.datastructures import Mesh
from compas.geometry import Line
from compas.geometry import intersection_line_line_xy, is_intersection_line_line_xy
from compas.geometry import distance_point_point
from compas_ghpython.artists import MeshArtist


class Ceilingsystem:
    def __init__(self, no_layers, min_gap, len_prim, len_sec, omni, prim_wid_out, prim_hi_out, prim_wid_in, prim_hi_in,
                 sec_wi, sec_hi, prim_intv, prim_fal, prim_dedens, sec_intv, sec_intv_dev, skip):

        self.layer_no = no_layers
        self.gap_min = min_gap
        self.primary_length = len_prim
        self.secondary_length = len_sec
        self.omnidirectional = omni
        self.primary_board_width_outside = prim_wid_out
        self.primary_board_height_outside = prim_hi_out
        self.primary_board_width_inside = prim_wid_in
        self.primary_board_height_inside = prim_hi_in
        self.primary_board_outside_dimensions = [self.primary_board_width_outside, self.primary_board_height_outside, self.primary_length]
        self.primary_board_inside_dimensions = [self.primary_board_width_inside, self.primary_board_height_inside, self.primary_length]
        self.secondary_board_width = sec_wi
        self.secondary_board_height = sec_hi
        self.secondary_board_dimensions = [self.secondary_board_width, self.secondary_board_height, self.secondary_length]

        self.primary_interval = prim_intv
        self.primary_falloff = prim_fal
        self.primary_dedensification = prim_dedens
        self.secondary_interval = sec_intv
        self.secondary_interval_development = sec_intv_dev
        self.skipping = skip

        self.primary_direction = 0
        self.secondary_direction = 1
        self.robot_origin_primary_direction = 0
        self.robot_origin_secondary_direction = 0
        self.robot_origin_z_direction = 0

        self.ceiling_grids = []
        self.timberboards = []

    # adds more parameters to control the orientation and location of the ceiling before starting the creation
    def robot_setup(self, prim_dir, sec_dir, rob_prim, rob_sec, rob_z):
        self.primary_direction = prim_dir
        self.secondary_direction = sec_dir
        self.robot_origin_primary_direction = rob_prim
        self.robot_origin_secondary_direction = rob_sec
        self.robot_origin_z_direction = rob_z
        self.standard_ceiling_setup()

    def __grid_creation(self):
        # in case the timber boards on the inside are the same size as usual
        if self.primary_board_height_inside < 0 or self.primary_board_width_inside < 0:
            primary_span_board_width_inside = self.primary_board_width_outside
            self.primary_board_height_inside = self.primary_board_height_outside

        # check for minimal distance
        # -.1 in order to be on the safe side
        self.gap_min -= .1
        if self.primary_board_height_outside < self.gap_min or self.primary_board_height_inside < self.gap_min or \
            (self.primary_interval - self.primary_board_width_outside) < self.gap_min or \
            (self.primary_interval - self.primary_board_width_inside) < self.gap_min:
            print("Too little gap")
            return 1

        # side_dedensification_intensity = how many elements do we kick out
        self.primary_dedensification *= 1000.0

        # define the room
        centre_ratio = (self.secondary_length - self.primary_falloff * 2) / self.secondary_length

        primary_graph_centre_line_x = self.secondary_length / 2

        # go into the borders
        # 1000 just so that we get nicer numbers
        primary_graph_default_ascent = 1000.0 / self.primary_interval
        primary_graph_centre_line_y = (primary_graph_centre_line_x // self.primary_interval) * 1000.0 - self.primary_dedensification
        primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio * primary_graph_centre_line_x

        # adjust the centre ratio in a way the endpoints fit perfectly
        inacc = (primary_graph_centre_line_x - primary_graph_centre_minimum_x) % self.primary_interval
        target = primary_graph_centre_minimum_x + inacc
        centre_ratio = (target - primary_graph_centre_line_x) / -primary_graph_centre_line_x

        # now start from scratch now with a different ratio
        primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio * primary_graph_centre_line_x
        primary_graph_centre_maximum_x = primary_graph_centre_line_x + centre_ratio * primary_graph_centre_line_x
        no_elements = (primary_graph_centre_line_x - primary_graph_centre_minimum_x) // self.primary_interval
        primary_graph_centre_minimum_y = primary_graph_centre_line_y - 1000.0 * no_elements
        primary_graph_centre_maximum_y = primary_graph_centre_line_y + 1000.0 * no_elements
        secondary_graph_y_maximum = primary_graph_centre_line_y * 2.0

        # math function for later
        g = primary_graph_centre_minimum_x*1.0
        d = primary_graph_centre_minimum_y
        s = primary_graph_default_ascent
        b = (2.0 * d) / g - s
        a = (s - b) / (2.0 * g)

        if b < 0:
            print("Error: Too extreme value!")

        primary_span_grid = []

        def graph_function(y):
            # in central part
            if y >= primary_graph_centre_minimum_y and y <= primary_graph_centre_maximum_y:
                x_value = primary_graph_centre_minimum_x + self.primary_interval * (y - primary_graph_centre_minimum_y) / 1000
                return x_value
            # error
            elif y > primary_graph_centre_line_y * 2 or y < 0:
                return 2
            # on the one hand; recalls the own function in a mirrored way and executes the last part of the function
            elif y > primary_graph_centre_maximum_y:
                # mirror everything to be on the safe side
                y = secondary_graph_y_maximum - y
                x_value = graph_function(y)
                return self.secondary_length - x_value
            # it's outside the centre: do the differential
            else:
                # unexplainable math function; derived from differential
                x_value = ((b * -1) + math.sqrt(b ** 2 - 4 * a * (-y))) / (2 * a)
                if x_value < 1:
                    x_value = self.primary_board_width_outside/2
                return x_value

        # +1 is an unelegant solution that makes sure the highest element is still respected
        for i in range(0, int(secondary_graph_y_maximum) + 1, 1000):
            result = graph_function(i)
            if result == 2:
                break
            else:
                primary_span_grid.append(result)
        # return primary_span_grid

        ############################################################
        ############################################################
        ############################################################
        # now comes the long span
        secondary_span_centre = self.primary_length // 2

        # helps to determine the graph function
        def longspan_function(val, searched):
            if searched == "y":
                return val ** self.secondary_interval_development
            if searched == "x":
                value = val ** (1 / self.secondary_interval_development)
                return value

        no_secondary_span_elements = self.secondary_length // self.secondary_interval
        # determine the steps
        secondary_graph_y_max = longspan_function(secondary_span_centre, "y")
        secondary_graph_y_min = secondary_graph_y_max * -1
        secondary_graph_y_step = secondary_graph_y_max / ((no_secondary_span_elements - 1) / 2)
        # print(secondary_graph_y_min)

        # go through the function to get the final values
        secondary_graph_y_current = 0
        secondary_graph_y_list_positive = []
        secondary_graph_y_list_final = []
        while secondary_graph_y_current <= secondary_graph_y_max + 1:
            # function only works for positive values!
            secondary_graph_y_list_positive.append(longspan_function(secondary_graph_y_current, "x"))
            secondary_graph_y_current += secondary_graph_y_step

        # now create a negative list
        for i in range(len(secondary_graph_y_list_positive) - 1, 0, -1):
            secondary_graph_y_list_final.append(secondary_graph_y_list_positive[i] * -1)
        # and now fuse them
        secondary_graph_y_list_final += secondary_graph_y_list_positive
        # and now push it by half the room width
        for val in range(len(secondary_graph_y_list_final)):
            secondary_graph_y_list_final[val] += self.primary_length / 2

        # print(secondary_graph_y_list_final)

        secondary_span_grid = [[], [], []]

        for element in range(1, len(secondary_graph_y_list_final) - 1, 2):
            secondary_span_grid[0].append(secondary_graph_y_list_final[element])
            secondary_span_grid[1].append(secondary_graph_y_list_final[element])
        for element in range(2, len(secondary_graph_y_list_final) - 1, 2):
            secondary_span_grid[0].append(secondary_graph_y_list_final[element])
            secondary_span_grid[2].append(secondary_graph_y_list_final[element])
        for j in range(len(secondary_span_grid)):
            secondary_span_grid[j].insert(0, self.secondary_board_width/2)
            secondary_span_grid[j].append(self.primary_length - self.secondary_board_width/2)

        secondary_span_grid[0].sort()
        return primary_span_grid, secondary_span_grid

    # creates the timber boards and equips them with dimensions and z-location
    def __data_setup(self):
        z_value = 0
        global_counter = 0
        for layer in range(self.layer_no):
            self.timberboards.append([])
            # outside layer, primary_span
            if layer == 0 or layer == self.layer_no - 1:
                boardheight = self.primary_board_height_outside
                z_value += boardheight
                for i in range(len(self.ceiling_grids[0])):
                    self.timberboards[layer].append(self.Timberboard(layer, global_counter, i, self.primary_board_outside_dimensions,
                                                                z_value, self.primary_direction, self.secondary_direction))
                    global_counter += 1
            elif layer % 2 == 0:
                boardheight = self.primary_board_height_inside
                z_value += boardheight
                # no dedensification on the inside
                if not self.skipping:
                    for i in range(len(self.ceiling_grids[0])):
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, i, self.primary_board_inside_dimensions,
                                                                    z_value, self.primary_direction, self.secondary_direction))
                        global_counter += 1
                # dedensification on the inside
                else:
                    # not sure about the -1
                    max_i = 0
                    for i in range(0, len(self.ceiling_grids[0]), 2):
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, i, self.primary_board_inside_dimensions,
                                                                    z_value, self.primary_direction, self.secondary_direction))
                        global_counter += 1
                        max_i = i
                    if len(self.ceiling_grids[0]) % 2 != 1:
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, max_i+1, self.primary_board_inside_dimensions,
                                                                z_value, self.primary_direction, self.secondary_direction))
                        global_counter += 1
            else:
                boardheight = self.secondary_board_height
                z_value += boardheight
                for i in range(0, len(self.ceiling_grids[1][0])):
                    self.timberboards[layer].append(self.Timberboard(layer, global_counter, i, self.secondary_board_dimensions,
                                                                z_value, self.secondary_direction, self.primary_direction))
                    global_counter += 1

    def grasshopper_draw(self):
        visualisations = []
        for woodLayer in self.timberboards:
            for board in woodLayer:
                my_box = board.box
                mesh_box = Mesh.from_shape(my_box)
                artist = MeshArtist(mesh_box)
                box_visualisation = artist.draw_mesh()
                visualisations.append(box_visualisation)
        return visualisations

    # specifies the centre point locations
    def __geometry_setup(self):
        for timber_layer in self.timberboards:
            for board in timber_layer:
                primary_direction_coordinate = self.primary_length / 2
                if board.layer % 2 == 0:
                    if self.skipping and board.layer != 0 and board.layer != self.layer_no:
                        # unnecessary branch at the moment but we might still need it so.....
                        secondary_direction_coordinate = self.ceiling_grids[0][board.no_in_layer]
                    else:
                        secondary_direction_coordinate = self.ceiling_grids[0][board.no_in_layer]
                else:
                    secondary_direction_coordinate = self.secondary_length/2
                    primary_direction_coordinate = self.ceiling_grids[1][0][board.no_in_layer]

                # now correct the location in reference to the robot
                primary_direction_coordinate -= self.robot_origin_primary_direction
                secondary_direction_coordinate -= self.robot_origin_secondary_direction
                z_direction_coordinate = board.z_drop - self.robot_origin_z_direction

                board.centre_point[self.primary_direction] = primary_direction_coordinate
                board.centre_point[self.secondary_direction] = secondary_direction_coordinate
                board.centre_point[2] = z_direction_coordinate - board.height
                board.box_update()

    # default setup without the robot stuff
    def standard_ceiling_setup(self):
        # calculates the basic centre lines
        # basically just a list of numbers, the rest comes later
        self.timberboards = []
        self.ceiling_grids = []
        self.ceiling_grids = self.__grid_creation()
        self.__data_setup()
        self.__geometry_setup()
        self.gluepoints()

    # subclass: the individual timber boards
    class Timberboard:
        def __init__(self, board_layer, identification, board_no_in_layer, board_dimensions,
                     z_value_toppoint, length_dir, width_dir):
            self.index = identification
            self.layer = board_layer
            self.no_in_layer = board_no_in_layer
            self.dimensions = board_dimensions
            self.width = self.dimensions[0]
            self.height = self.dimensions[1]
            self.length = self.dimensions[2]
            self.drop_point = Point(0,0,0)
            self.centre_point = Point(0,0,0)
            self.z_drop = z_value_toppoint
            self.length_direction = length_dir
            self.width_direction = width_dir
            self.glue_givers = []
            self.glue_receivers = []
            self.receiving_neighbours = []
            self.giving_neighbours = []

            if self.length_direction == 0:
                self.length_vector = Vector(1, 0, 0)
                self.width_vector = Vector(0, 1, 0)
            else:
                self.length_vector = Vector(0, 1, 0)
                self.width_vector = Vector(1, 0, 0)

            self.board_frame = Frame(self.centre_point, self.length_vector, self.width_vector)
            self.box = Box(self.board_frame, self.length, self.width, self.height)

        def box_update(self):
            self.board_frame = Frame(self.centre_point, self.length_vector, self.width_vector)
            self.box = Box(self.board_frame, self.length, self.width, self.height)

    # creates all the gluepoints between the boards and specifies neighbour relationships
    def gluepoints(self):
        def board_intersection(pt1, vec1, len1, pt2, vec2, len2):
            line1 = line_creator(pt1, vec1, len1)
            line2 = line_creator(pt2, vec2, len2)
            # to check whether the boards are parallel
            if vec1 != vec2:
                int_pt = intersection_line_line_xy(line1, line2)
            else:
                # expand here later to deal with gluing parallel boards
                return 0
            # since intersection also hits when the lines intersect in their continuation, we have to add that one
            if distance_point_point(pt1, int_pt) < len1/2 and \
                distance_point_point(pt2, int_pt) < len2/2:
                return int_pt
            else:
                return 0

        def line_creator(pt_a, vec, length):
            pt_b = pt_a + scale_vector(vec, length/2)
            pt_a = pt_a - scale_vector(vec, length/2)
            return pt_a, pt_b

        for layer_number in range(1,self.layer_no):
            for board in self.timberboards[layer_number]:
                for other_board in self.timberboards[layer_number - 1]:
                    gluept = board_intersection(board.centre_point, board.length_vector, board.length,
                                                other_board.centre_point, other_board.length_vector, other_board.length)
                    # if there simply is no intersection
                    if gluept == 0:
                        continue
                    board.glue_givers.append(gluept)
                    other_board.glue_receivers.append(gluept)
                    board.receiving_neighbours.append(other_board)
                    other_board.giving_neighbours.append(board)


# secondary_span_interval_development: 1 = constant, <1: denser in the centre, >1: denser on the edges
# operable range approximately 0.6/6

layer_no = 5
gap_min = 4.0
primary_length = 450.0
secondary_length = 840.0
omnidirectional = True
primary_board_width_outside = 6.0
primary_board_height_outside = 4.0
primary_board_width_inside = 6.0
primary_board_height_inside = 4.0
primary_board_outside_dimensions = [primary_board_width_outside, primary_board_height_outside, primary_length]
primary_board_inside_dimensions = [primary_board_width_inside, primary_board_height_inside, primary_length]
secondary_board_width = 4.0
secondary_board_height = 4.0
secondary_board_dimensions = [secondary_board_width, secondary_board_height, secondary_length]

primary_interval = 12.0
primary_falloff = 90.0
primary_dedensification = 3.0
secondary_interval = 80.0
secondary_interval_development = .7
skip_centrals = True

primary_direction = 0
secondary_direction = 0
robot_origin_primary_direction = 0
robot_origin_secondary_direction = 0
robot_origin_z_direction = 0


myCeiling = Ceilingsystem(layer_no, gap_min, primary_length, secondary_length, omnidirectional,
                          primary_board_width_outside, primary_board_height_outside,
                          primary_board_width_inside, primary_board_height_inside,
                          secondary_board_width, secondary_board_height,
                          primary_interval, primary_falloff, primary_dedensification,
                          secondary_interval, secondary_interval_development, skip_centrals)

myCeiling.robot_setup(primary_direction, secondary_direction, robot_origin_primary_direction,
                      robot_origin_secondary_direction, robot_origin_z_direction)

myCeiling.gluepoints()
print("done")


