from __future__ import division
from __future__ import absolute_import
from __future__ import print_function

from assembly_information_model.assembly import Assembly

from compas.geometry import Frame
from compas.geometry import Vector, Point
from compas.geometry import Box, Frame, Vector, scale_vector, normalize_vector, \
    Polygon, Rotation, angle_vectors_signed, Transformation

from compas.geometry import intersection_line_line_xy, distance_point_point, \
    is_point_in_polygon_xy, rotate_points, rotate_points_xy
from compas.geometry import distance_point_point_xy

from compas.geometry import Transformation
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import distance_point_point
from compas.datastructures import Network, mesh_offset

import math

# from timber_clay_hybrid.export import Exporter
from element_TCH import TCHElement


class TCHAssembly(Assembly):
    """A data structure for discrete element assemblies for human robot collaboration
    """

    def __init__(self,
                 layer_no_input, gap_min_input, primary_length_input, secondary_length_input, omnidirectional_input, primary_board_width_outside_input,
                 primary_board_height_outside_input, primary_board_width_inside_input, primary_board_height_inside_input, secondary_board_width_input,
                 secondary_board_height_input, primary_interval_input, primary_falloff_input, primary_dedensification_input, secondary_interval_input,
                 secondary_interval_development_input, skip_centrals_input, origin_frame_input, prim_vert_sup_input=False, sec_vert_sup_input=False,
                 vertical_support_interlock_input=None, vertical_support_width_input=None, flip_toolframe_prim_input=False, flip_toolframe_sec_input=False):

        super(TCHAssembly, self).__init__()

        self.layer_no = layer_no_input
        self.gap_min = gap_min_input
        self.primary_length = primary_length_input
        self.secondary_length = secondary_length_input
        self.omnidirectional = omnidirectional_input
        self.primary_board_width_outside = primary_board_width_outside_input
        self.primary_board_height_outside = primary_board_height_outside_input
        self.primary_board_width_inside = primary_board_width_inside_input
        self.primary_board_height_inside = primary_board_height_inside_input
        self.primary_board_outside_dimensions = [self.primary_board_width_outside,
                                                 self.primary_board_height_outside, self.primary_length]
        self.primary_board_inside_dimensions = [self.primary_board_width_inside,
                                                self.primary_board_height_inside, self.primary_length]
        self.secondary_board_width = secondary_board_width_input
        self.secondary_board_height = secondary_board_height_input
        self.secondary_board_dimensions = [self.secondary_board_width, self.secondary_board_height,
                                           self.secondary_length]
        self.primary_interval = primary_interval_input
        if self.omnidirectional and primary_falloff_input and primary_dedensification_input:
            self.primary_falloff = primary_falloff_input
            self.primary_dedensification = primary_dedensification_input
        else:
            self.primary_falloff = 0.0
            self.primary_dedensification = 0
        self.secondary_interval = secondary_interval_input
        self.secondary_interval_development = secondary_interval_development_input
        self.skipping = skip_centrals_input
        self.primary_direction = 0
        self.secondary_direction = 1
        self.origin_fr = origin_frame_input
        self.origin_pt = self.origin_fr[0]
        self.prim_dir = self.origin_fr[1]
        self.sec_dir = self.origin_fr[2]
        self.sec_fr = Frame(self.origin_pt, self.sec_dir, self.prim_dir)
        self.flip_toolframe_prim = flip_toolframe_prim_input
        self.flip_toolframe_sec = flip_toolframe_sec_input

        self.setup_done = False

        # Vertical Support
        self.prim_vert_sup = prim_vert_sup_input
        self.sec_vert_sup = sec_vert_sup_input
        self.vertical_support_width = vertical_support_width_input
        self.vertical_support_interlock = vertical_support_interlock_input

        # ADVANCED PARAMETERS
        self.advanced_setup = False
        self.gluepathwidth = None

    # default setup that collects the relevant functions
    def floorslab_geometry_creation(self):
        # calculates the basic centre lines
        # basically just a list of numbers, the rest comes later
        def __grid_creation():
            # in case the timber boards on the inside are the same size as usual
            if self.primary_board_height_inside < 0 or self.primary_board_width_inside < 0:
                primary_span_board_width_inside = self.primary_board_width_outside
                self.primary_board_height_inside = self.primary_board_height_outside

            # check for minimal distance
            # -.1 in order to be on the safe side
            self.gap_min -= .001
            if self.primary_board_height_outside < self.gap_min or self.primary_board_height_inside < self.gap_min or \
                (self.primary_interval - self.primary_board_width_outside) < self.gap_min or \
                (self.primary_interval - self.primary_board_width_inside) < self.gap_min:
                print("Too little gap")
                return 1

            # side_dedensification_intensity = how many elements do we kick out
            self.primary_dedensification *= 1000.0

            if self.primary_dedensification == 0:
                my_inacc = (self.secondary_length - self.primary_board_width_outside) % self.primary_interval
                no_intervals = (self.secondary_length - self.primary_board_width_outside) // self.primary_interval
                self.primary_interval += my_inacc / no_intervals

            # define the room
            centre_ratio = (self.secondary_length - self.primary_falloff * 2) / self.secondary_length
            primary_graph_centre_line_x = self.secondary_length / 2

            # go into the borders
            # 1000 just so that we get nicer numbers
            primary_graph_default_ascent = 1000.0 / self.primary_interval
            primary_graph_centre_line_y = (primary_graph_centre_line_x // self.primary_interval) * 1000.0 - self.primary_dedensification
            primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio * primary_graph_centre_line_x

            # adjust the centre ratio in a way the endpoints fit perfectly
            # not sure what this was supposed to do precisely, didn't work in this instance
            inacc = (primary_graph_centre_line_x - primary_graph_centre_minimum_x) % self.primary_interval
            target = primary_graph_centre_minimum_x + inacc

            centre_ratio = (target - primary_graph_centre_line_x) / -primary_graph_centre_line_x

            # now start from scratch now with a different ratio
            primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio * primary_graph_centre_line_x
            primary_graph_centre_maximum_x = primary_graph_centre_line_x + centre_ratio * primary_graph_centre_line_x

            # +.1 is a bit of a dirty fix against float inaccuracies
            no_elements = ((primary_graph_centre_line_x - primary_graph_centre_minimum_x) + 0.001) // self.primary_interval

            primary_graph_centre_minimum_y = primary_graph_centre_line_y - 1000.0 * no_elements
            primary_graph_centre_maximum_y = primary_graph_centre_line_y + 1000.0 * no_elements
            secondary_graph_y_maximum = primary_graph_centre_line_y * 2.0

            # math function for later
            falloff_calculation_factor = 2.0
            b = -1
            g = primary_graph_centre_minimum_x * 1.0
            d = primary_graph_centre_minimum_y
            s = primary_graph_default_ascent

            # loop is necessary because of the different falloff setups; without it, there are sometimes no boards on the ends
            if self.primary_dedensification > 1:
                while b < 0:
                    b = (falloff_calculation_factor * d) / g - s
                    falloff_calculation_factor += 0.05
            else:
                b = (falloff_calculation_factor * d) / g - s
            a = (s - b) / (falloff_calculation_factor * g)

            primary_span_grid = [[], []]

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
                        x_value = self.primary_board_width_outside / 2
                    return x_value

            # now we create the primary span grid
            # +1 is an inelegant solution that makes sure the highest element is still respected
            for i in range(0, int(secondary_graph_y_maximum) + 1, 1000):
                # omnidirectional case
                if self.primary_dedensification > 0:
                    result = graph_function(i)
                    if result == 2:
                        break
                # monodirectional case
                else:
                    result = self.primary_board_width_outside / 2 + (i / 1000) * self.primary_interval
                primary_span_grid[0].append(result)

            # dirty solution because otherwise the last board would have been skipped
            if self.primary_dedensification == 0:
                primary_span_grid[0].append(self.secondary_length - self.primary_board_width_outside / 2)
            # another little dirty fix because the last board isn't always skipped by default and then we have one too many
            if abs(primary_span_grid[0][-1] - primary_span_grid[0][-2]) < self.primary_board_width_outside:
                primary_span_grid[0].pop(-2)

            # now create also the alternative lines in the gaps of the grid
            if self.primary_dedensification > 0:
                for i in range(500, int(secondary_graph_y_maximum) + 1, 1000):
                    result = graph_function(i)
                    if result == 2:
                        break
                    else:
                        primary_span_grid[1].append(result)
            else:
                for i in range(len(primary_span_grid[0]) - 1):
                    new_entry = (primary_span_grid[0][i] + primary_span_grid[0][i + 1]) / 2
                    primary_span_grid[1].append(new_entry)

            # now a safety thing to make sure that there is a position at the ends
            if primary_span_grid[0][0] > self.primary_board_width_outside / 1.9:
                print("Warning: No board on the sides")

            ############################################################
            ############################################################
            ############################################################
            # now comes the long span
            secondary_span_centre = self.primary_length / 2
            # correction for the interval
            my_inacc = self.primary_length % self.secondary_interval
            no_intervals = self.primary_length // self.secondary_interval
            self.secondary_interval += my_inacc / no_intervals

            # helps to determine the graph function
            def longspan_function(val, searched):
                if searched == "y":
                    return val ** self.secondary_interval_development
                if searched == "x":
                    value = val ** (1 / self.secondary_interval_development)
                    return value

            # careful, actually there should be one more, might explain some trouble later
            no_secondary_span_elements = self.primary_length // self.secondary_interval
            # determine the steps
            secondary_graph_y_max = longspan_function(secondary_span_centre, "y")
            secondary_graph_y_step = secondary_graph_y_max / (no_secondary_span_elements / 2)

            # go through the function to get the final values
            secondary_graph_y_current = 0
            secondary_graph_y_list_positive = []
            secondary_graph_y_list_final = []
            while secondary_graph_y_current <= secondary_graph_y_max + .01:
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

            secondary_span_grid = [set(), set(), set()]

            for element in range(0, len(secondary_graph_y_list_final), 2):
                secondary_span_grid[0].add(secondary_graph_y_list_final[element])
                secondary_span_grid[1].add(secondary_graph_y_list_final[element])
            for element in range(1, len(secondary_graph_y_list_final), 2):
                secondary_span_grid[0].add(secondary_graph_y_list_final[element])
                secondary_span_grid[2].add(secondary_graph_y_list_final[element])

            for j in range(len(secondary_span_grid)):
                secondary_span_grid[j] = list(secondary_span_grid[j])
                secondary_span_grid[j].sort()
                secondary_span_grid[j][0] = self.secondary_board_width / 2
                secondary_span_grid[j][len(secondary_span_grid[j]) - 1] = self.primary_length - self.secondary_board_width / 2

            print(secondary_span_grid)

            return primary_span_grid, secondary_span_grid

        # adapts the advanced boards to meaningful lengths
        def __advanced_length_correction():
            # inside shear supports
            if self.primary_inside_support_length > 0.0:
                position = 0
                while self.primary_inside_support_length > self.floorslab_grids[1][0][position]:
                    position += 1
                # //1 just to get a nicer number
                self.primary_inside_support_length = ((self.floorslab_grids[1][0][position] + self.secondary_board_width / 2)
                                                      + self.secondary_board_width / 2) // 1
                self.primary_inside_support_dimensions[2] = self.primary_inside_support_length

            # outside momentum supports
            if self.primary_outside_support_length > 0.0:
                outside_grid = self.floorslab_grids[1][0]
                outside_centre_id = len(outside_grid) // 2
                position = 1
                while outside_grid[outside_centre_id + position] - outside_grid[outside_centre_id - position] < self.primary_outside_support_length:
                    position += 1
                # //1 just to get a nicer number
                self.primary_outside_support_length = ((outside_grid[outside_centre_id + position] - outside_grid[outside_centre_id - position]) +
                                                       self.secondary_board_width) // 1 + .01
                self.primary_outside_support_dimensions[2] = self.primary_outside_support_length

        # makes sure that the program doesn't crash when the user/algorithm enters some nonsense
        def input_check():
            input_validity = True
            # check for too much dedensification
            if self.primary_dedensification > 0:
                if self.primary_falloff / self.primary_interval - self.primary_dedensification < 1:
                    input_validity = False
                    print("Too much dedensification")

            # check whether the interval is realistic
            if self.primary_dedensification == 0:
                interval_intolerance = (self.secondary_length - self.primary_board_width_outside) % self.primary_interval
                no_intervals = (self.secondary_length - self.primary_board_width_outside) // self.primary_interval
                if interval_intolerance > 0.1:
                    interval_intolerance /= no_intervals
                    self.primary_interval += interval_intolerance
            return input_validity

        def vert_sup_setup(prim_board_sup=False, sec_board_sup=False, available_lengths=None, gap_tolerance=0.2, vert_gap_min=5.0):
            self.prim_vert_sup = prim_board_sup
            self.sec_vert_sup = sec_board_sup
            self.vert_sup_lengths = available_lengths
            self.vert_sup_gap_tolerance = gap_tolerance
            self.vert_sup_gap_min = vert_gap_min

        # to do the sorting
        def __lt__(self, other):
            return self.grid_position < other.grid_position

        # creates the timber board instances and equips them with dimensions and z-location
        # momentum support pieces missing now
        def __board_data_setup(advanced):

            def board_element_setup(dimensions, lay, glob_count, lay_count, z_val, len_vec, wid_vec, grid_pos, vert_sup=False, perp=False, loc="centre"):
                my_width = dimensions[0]
                my_height = dimensions[1]
                my_length = dimensions[2]

                # very wide pieces that both carry vertical load and connect to the other boards
                if vert_sup and not perp:
                    if lay_count == 0:
                        grid_pos -= my_width / 2
                        grid_pos += (self.vertical_support_interlock - self.vertical_support_width) / 2
                    else:
                        grid_pos += my_width / 2
                        grid_pos -= (self.vertical_support_interlock - self.vertical_support_width) / 2
                    my_width = self.vertical_support_interlock + self.vertical_support_width
                    if self.prim_vert_sup and self.sec_vert_sup:
                        my_length += self.vertical_support_width * 2
                # not so wide pieces that only carry vertical load
                elif perp:
                    if self.prim_vert_sup and self.sec_vert_sup:
                        my_length -= self.vertical_support_interlock * 2
                    my_width = self.vertical_support_width

                if lay % 2 == 0:
                    my_frame = self.origin_fr
                    layer_standard_length = self.primary_length
                else:
                    my_frame = self.sec_fr
                    layer_standard_length = self.secondary_length

                my_dir1 = normalize_vector(my_frame[1])
                my_dir2 = normalize_vector(my_frame[2])
                dist = grid_pos

                if loc == "high":
                    if not perp:
                        length_attribute_1 = layer_standard_length - my_length / 2
                    else:
                        length_attribute_1 = layer_standard_length + my_width / 2
                elif loc == "low":
                    if not perp:
                        length_attribute_1 = my_length / 2
                    else:
                        length_attribute_1 = -my_width / 2
                else:
                    length_attribute_1 = layer_standard_length / 2

                # position parallel to the boards (if not sup)
                my_vec1 = scale_vector(my_dir1, length_attribute_1)
                # position perpendicular to board direction (if not sup)
                my_vec2 = scale_vector(my_dir2, dist)
                # height vector
                # potential error here, z might be wrong
                my_vec3 = Vector(0, 0, z_val - my_height / 2)
                my_centre = self.origin_pt + my_vec1 + my_vec2 + my_vec3
                my_centre_point = my_centre

                my_drop_point = my_centre + Vector(0, 0, my_height / 2)

                if not perp:
                    my_length_vector = Vector(my_dir1[0], my_dir1[1], my_dir1[2])
                    my_width_vector = Vector(my_dir2[0], my_dir2[1], my_dir2[2])
                else:
                    my_length_vector = Vector(my_dir2[0], my_dir2[1], my_dir2[2])
                    my_width_vector = Vector(my_dir1[0], my_dir1[1], my_dir1[2])

                old_centre = Point(my_centre[0], my_centre[1], my_centre[2])
                my_vec = Vector.from_start_end(old_centre, my_centre)

                # self.network.node[my_board.global_count]['x'] = my_centre[0]
                # self.network.node[my_board.global_count]['y'] = my_centre[1]
                # self.network.node[my_board.global_count]['z'] = my_centre[2]

                my_board_frame = Frame(my_centre_point, my_length_vector, my_width_vector)
                if (lay % 2 != 0 and not perp) or (lay % 2 == 0 and perp):
                    if self.flip_toolframe_sec:
                        my_tool_frame = Frame(my_drop_point, my_length_vector, my_width_vector * -1)
                    else:
                        my_tool_frame = Frame(my_drop_point, my_length_vector * -1, my_width_vector)
                else:
                    my_tool_frame = Frame(my_drop_point, my_length_vector, my_width_vector)
                my_box = Box(my_board_frame, my_length, my_width, my_height)

                self.add_element(TCHElement(my_board_frame, [my_width, my_height, my_length], lay, glob_count, lay_count, grid_pos, my_tool_frame, my_box, vert_sup, perp))

            z_value = 0
            global_counter = 0
            for layer in range(self.layer_no):
                layer_counter = 0
                # outside layer, primary_span
                if layer == 0 or (layer == self.layer_no - 1 and layer % 2 == 0):
                    boardheight = self.primary_board_height_outside
                    z_value += boardheight
                    # main part
                    for i in range(len(self.floorslab_grids[0][0])):
                        if self.sec_vert_sup and (i == 0 or i == len(self.floorslab_grids[0][0]) - 1):
                            vert_sup = True
                        else:
                            vert_sup = False

                        board_position = self.floorslab_grids[0][0][i]
                        board_element_setup(self.primary_board_outside_dimensions, layer, global_counter, layer_counter, z_value,
                                            self.primary_direction, self.secondary_direction, board_position, vert_sup)
                        global_counter += 1
                        layer_counter += 1

                    # inserts the momentum support pieces if it's wished
                    last_piece = self.network.node[global_counter - 1]["element"]
                    last_grid_pos = last_piece.grid_position
                    last_layer_pos = last_piece.no_in_layer

                    if self.advanced_setup and self.primary_outside_support_length > 0:
                        for i in range(global_counter - 1, global_counter - last_layer_pos - 1, -1):
                            current_layer_counter = self.network.node[i]["element"].no_in_layer - 1
                            # check for the layer
                            if (self.network.node[i]["element"].grid_position - self.network.node[i - 1]["element"].grid_position >
                                self.primary_outside_support_gap_min and
                                (self.network.node[i]["element"].grid_position - self.network.node[i - 1]["element"].grid_position) / 2 -
                                self.primary_board_width_outside > self.gap_min and
                                self.floorslab_grids[0][1][current_layer_counter] > self.primary_outside_support_distance_to_edge_min and
                                self.secondary_length - self.floorslab_grids[0][1][current_layer_counter] > self.primary_outside_support_distance_to_edge_min and
                                self.floorslab_grids[0][1][current_layer_counter] < self.primary_outside_support_distance_to_edge_max and
                                self.secondary_length - self.floorslab_grids[0][1][current_layer_counter] < self.primary_outside_support_distance_to_edge_max and
                                (last_piece.layer in self.primary_outside_support_layers or self.primary_outside_support_layers == [])
                            ):
                                dims = [self.primary_board_outside_dimensions[0], self.primary_board_outside_dimensions[1],
                                        self.primary_outside_support_length]
                                board_element_setup(dims, layer, global_counter, layer_counter, z_value,
                                                    self.primary_direction, self.secondary_direction, self.floorslab_grids[0][1][current_layer_counter], False)

                                global_counter += 1
                                layer_counter += 1

                    if self.prim_vert_sup:
                        dims = [self.primary_board_outside_dimensions[0], self.primary_board_outside_dimensions[1],
                                self.secondary_board_dimensions[2]]

                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.primary_direction, self.secondary_direction,
                                            self.secondary_length / 2, True, True, "low")
                        global_counter += 1
                        layer_counter += 1
                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.primary_direction, self.secondary_direction,
                                            self.secondary_length / 2, True, True, "high")
                        global_counter += 1
                        layer_counter += 1

                # inside layer, primary span
                elif layer % 2 == 0:
                    boardheight = self.primary_board_height_inside
                    z_value += boardheight
                    # no dedensification on the inside
                    print(self.skipping)
                    if self.skipping < 2:
                        for i in range(len(self.floorslab_grids[0][0])):
                            if self.sec_vert_sup and (i == 0 or i == len(self.floorslab_grids[0][0]) - 1):
                                vert_sup = True
                            else:
                                vert_sup = False

                            board_element_setup(self.primary_board_inside_dimensions, layer, global_counter, i, z_value,
                                                self.primary_direction, self.secondary_direction,
                                                self.floorslab_grids[0][0][i], vert_sup)
                            global_counter += 1

                    # dedensification on the inside
                    else:
                        # not sure about the -1
                        for i in range(0, len(self.floorslab_grids[0][0]), self.skipping):
                            if self.sec_vert_sup and (i == 0 or i == len(self.floorslab_grids[0][0]) - 1):
                                vert_sup = True
                            else:
                                vert_sup = False
                            board_element_setup(self.primary_board_inside_dimensions, layer, global_counter, layer_counter,
                                                z_value, self.primary_direction, self.secondary_direction,
                                                self.floorslab_grids[0][0][i], vert_sup)
                            global_counter += 1
                            layer_counter += 1

                        # last element??
                        if len(self.floorslab_grids[0][0]) % 2 != 1:
                            if self.sec_vert_sup:
                                vert_sup = True
                            else:
                                vert_sup = False
                            board_element_setup(self.primary_board_inside_dimensions, layer, global_counter, layer_counter,
                                                z_value, self.primary_direction, self.secondary_direction,
                                                self.floorslab_grids[0][0][-1], vert_sup)

                            global_counter += 1
                            layer_counter += 1

                        # inserts the shear support pieces if wished
                        if self.advanced_setup and self.primary_inside_support_length > 0 and self.skipping:
                            iteration_counter = 0
                            fixed_global_counter = global_counter
                            for i in range(len(self.floorslab_grids[0][0]) - 1, 1, -2):
                                iteration_counter += 1
                                current_layer_counter = self.network.node[i]["element"].no_in_layer - 1
                                # now check whether there should really be a board at that position
                                upper_element = self.network.node[fixed_global_counter - iteration_counter]["element"]
                                lower_element = self.network.node[fixed_global_counter - iteration_counter - 1]["element"]
                                if (upper_element.grid_position - lower_element.grid_position > self.primary_inside_support_gap_min and

                                    self.floorslab_grids[0][0][i] > self.primary_inside_support_distance_to_edge_min and
                                    self.secondary_length - self.floorslab_grids[0][0][i] > self.primary_inside_support_distance_to_edge_min and
                                    self.floorslab_grids[0][0][i] < self.primary_inside_support_distance_to_edge_max and
                                    self.secondary_length - self.floorslab_grids[0][0][i] < self.primary_inside_support_distance_to_edge_max and
                                    (self.primary_inside_support_layers == [] or layer in self.primary_inside_support_layers)):

                                    dims = [self.primary_board_inside_dimensions[0], self.primary_board_inside_dimensions[1],
                                            self.primary_inside_support_length]

                                    board_element_setup(dims, layer, global_counter, layer_counter, z_value,
                                                        self.primary_direction, self.secondary_direction,
                                                        self.floorslab_grids[0][0][current_layer_counter], False, loc="low")
                                    layer_counter += 1
                                    global_counter += 1

                                    board_element_setup(dims, layer, global_counter, layer_counter, z_value,
                                                        self.primary_direction, self.secondary_direction,
                                                        self.floorslab_grids[0][0][current_layer_counter], False, loc="high")
                                    global_counter += 1
                                    layer_counter += 1
                                else:
                                    continue

                    if self.prim_vert_sup:
                        dims = [self.primary_board_outside_dimensions[0], self.primary_board_inside_dimensions[1],
                                self.secondary_board_dimensions[2]]

                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.primary_direction, self.secondary_direction,
                                            self.secondary_length / 2, True, True, "low")
                        global_counter += 1
                        layer_counter += 1
                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.primary_direction, self.secondary_direction,
                                            self.secondary_length / 2, True, True, "high")
                        global_counter += 1
                        layer_counter += 1

                # secondary span
                else:
                    boardheight = self.secondary_board_height
                    z_value += boardheight
                    x = len(self.floorslab_grids[1][0])
                    for i in range(0, len(self.floorslab_grids[1][0])):
                        if self.prim_vert_sup and (i == 0 or i == len(self.floorslab_grids[1][0]) - 1):
                            vert_sup = True
                        else:
                            vert_sup = False

                        board_element_setup(self.secondary_board_dimensions, layer, global_counter, layer_counter,
                                            z_value, self.secondary_direction, self.primary_direction,
                                            self.floorslab_grids[1][0][i], vert_sup)

                        global_counter += 1
                        layer_counter += 1

                    if self.sec_vert_sup:
                        dims = [self.primary_board_outside_dimensions[0], self.secondary_board_dimensions[1],
                                self.primary_board_outside_dimensions[2]]

                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.secondary_direction, self.primary_direction,
                                            self.primary_length / 2, True, True, "low")

                        global_counter += 1
                        layer_counter += 1

                        board_element_setup(dims, layer, global_counter, layer_counter,
                                            z_value, self.secondary_direction, self.primary_direction,
                                            self.primary_length / 2, True, True, "high")
                        global_counter += 1
                        layer_counter += 1

        if input_check():
            self.floorslab_grids = []
            self.floorslab_grids = __grid_creation()
            if self.advanced_setup:
                __advanced_length_correction()
            __board_data_setup(self.advanced_setup)
            self.setup_done = True
            return self
        else:
            print("didn't pass input check")
            return 1

    def component_instances_export(self):
        global_boards = []
        for brd in self.elements():
            board = brd[1]
            board_format = (board.width, board.height)
            format_entry_complete = False
            length_entry_complete = False
            # now loop through the whole list to see whether your entry is already there

            for j, formats in enumerate(global_boards):
                # format[0][0] are the dimensions of the board
                # format[0][1] is the total length of the format
                if formats[0][0] == board_format:
                    format_entry_complete = True
                    # the format already exists

                    for i in range(1, len(formats)):
                        # the length already exists
                        if board.length == formats[i][0]:
                            formats[i][1] += 1
                            length_entry_complete = True
                            break
                    # if it didn't find an entry
                    if not length_entry_complete:
                        formats.append([board.length, 1])
                    formats[0][1] += board.length

            if not format_entry_complete:
                global_boards.append([[board_format, board.length], [board.length, 1]])

        print("[[[profile_width_1, profile_height_1], total_length_1], [length_a, no_pieces], [length_b, no_pieces], .....]"
              "[[profile_width_2, profile_height_2], total_length_2], [length_a, no_pieces], [length_b, no_pieces], .....]"
              "...]")
        return global_boards

    def assembly_creation(self, stack_origin_frame, max_hi=10, max_width=4, bottom_pickup=0, col_pickup=0,
                          distance_within_stack=0.08, distance_between_stacks=0.12, full_efficiency=True,
                          snake=False, gluepath_width=0.04, glue_station_default_frame=None, safety_distance=None, safety_distance_gluepoints=None, dryrun=False,
                          flipped_dropframe_distance=None, sorting_direction=1, sorting_flipped=False, bending_behaviour=None):

        # creates all the gluepoints between the boards and specifies neighbour relationships
        def gluepoints():
            def corner_point_finder(my_board):
                def sidepoints(pt):
                    left_pt = pt - scale_vector(my_board.width_vector, my_board.width / 2)
                    right_pt = pt + scale_vector(my_board.width_vector, my_board.width / 2)
                    return left_pt, right_pt

                upper_boarder_centre = my_board.centre_point + scale_vector(my_board.length_vector, my_board.length / 2)
                lower_boarder_centre = my_board.centre_point + scale_vector(my_board.length_vector, -my_board.length / 2)

                left_pt_up = upper_boarder_centre + scale_vector(my_board.width_vector, -my_board.width / 2)
                right_pt_up = upper_boarder_centre + scale_vector(my_board.width_vector, my_board.width / 2)
                left_pt_down = lower_boarder_centre + scale_vector(my_board.width_vector, -my_board.width / 2)
                right_pt_down = lower_boarder_centre + scale_vector(my_board.width_vector, my_board.width / 2)

                return [left_pt_up, right_pt_up, right_pt_down, left_pt_down]

            def line_maker(points):
                lines = []
                for i in range(len(points) - 1):
                    pt1 = (points[i][0], points[i][1], points[i][2])
                    pt2 = (points[i + 1][0], points[i + 1][1], points[i + 1][2])
                    lines.append((pt1, pt2))
                pt1 = (points[-1][0], points[-1][1], points[-1][2])
                pt2 = (points[0][0], points[0][1], points[0][2])
                lines.append((pt1, pt2))
                return lines

            def board_intersection(brd1, brd2):
                def surface_calc(upper_board, lower_board, intersection):
                    vec1 = Vector(lower_board.length_vector[0], lower_board.length_vector[1], lower_board.length_vector[2])
                    vec2 = Vector(upper_board.length_vector[0], upper_board.length_vector[1], upper_board.length_vector[2])
                    # in case there is a wide board at the top; whatever that exactly changes
                    if upper_board.width > .01:
                        ang = vec1.angle(vec2)
                    if vec1.angle(vec2) > 0.5:
                        len_intersection = lower_board.width
                        wid_intersection = upper_board.width
                    else:
                        # for now we assume that they are parallel in this case, potential error source for later, though
                        len_intersection = min(upper_board.length, lower_board.length)
                        wid_intersection = min(upper_board.width, lower_board.width)

                    dim1 = scale_vector(upper_board.length_vector, len_intersection * .5)
                    dim2 = scale_vector(upper_board.width_vector, wid_intersection * .5)

                    # this procedure is necessary only because of the glue path planning to make sure points are always ordered clockwise
                    ang = angle_vectors_signed(dim1, dim2, Vector(0, 0, 1))
                    if ang > 0:
                        pt1 = intersection + dim1 - dim2
                        pt2 = intersection + dim1 + dim2
                        pt3 = intersection - dim1 + dim2
                        pt4 = intersection - dim1 - dim2
                    else:
                        pt1 = intersection - dim1 + dim2
                        pt2 = intersection + dim1 + dim2
                        pt3 = intersection + dim1 - dim2
                        pt4 = intersection - dim1 - dim2

                    intersection_surf = Polygon([pt1, pt2, pt3, pt4])
                    return intersection_surf

                vec1 = brd1.frame[1]
                vec2 = brd2.frame[1]
                centre_point1 = brd1.frame[0]
                centre_point2 = brd2.frame[0]
                line1 = line_creator(centre_point1, vec1, brd1.length)
                line2 = line_creator(centre_point2, vec2, brd2.length)

                standard_option_enabled = False
                # to check whether the boards are parallel, if they are not then go into the first loop
                if ((brd1.vert_sup is False and brd2.vert_sup is False) or
                    (brd1.vert_sup and not brd1.perp and brd2.vert_sup and not brd2.perp)) and \
                    abs(vec1.angle(vec2)) > 0.1:
                    int_pt = intersection_line_line_xy(line1, line2)
                    if int_pt is None:
                        return None
                    # since intersection also hits when the lines intersect in their continuation, we have to add that one
                    if distance_point_point_xy(brd1.centre_point, int_pt) < brd1.length / 2 and \
                        distance_point_point_xy(brd2.centre_point, int_pt) < brd2.length / 2 and int_pt is not None and int_pt != 0:
                        intersection_point = Point(int_pt[0], int_pt[1], brd2.tool_frame[0][2])
                        int_srf = surface_calc(brd1, brd2, intersection_point)
                        return int_srf
                    else:
                        return None
                else:
                    # if they are not the default intersectors then follow this procedure
                    pts1 = corner_point_finder(brd1)
                    rec1 = Polygon(pts1)
                    line1 = line_maker(pts1)
                    pts2 = corner_point_finder(brd2)
                    rec2 = Polygon(pts2)
                    line2 = line_maker(pts2)
                    # calculate intersection points
                    intersects = set()
                    final_points = set()
                    segments = []

                    # unnecessary, just for checking
                    intsct_counter = 0
                    for l1 in line1:
                        for l2 in line2:
                            intersection_pt = intersection_line_line_xy(l1, l2)
                            if intersection_pt is not None:
                                # check that the intersection point is actually on the line or somewhere in the distance
                                line1_length = distance_point_point_xy(l1[0], l1[1])
                                int1_dist_1 = distance_point_point_xy(l1[0], intersection_pt)
                                int1_dist_2 = distance_point_point_xy(l1[1], intersection_pt)
                                line2_length = distance_point_point_xy(l2[0], l2[1])
                                int2_dist_1 = distance_point_point_xy(l2[0], intersection_pt)
                                int2_dist_2 = distance_point_point_xy(l2[1], intersection_pt)
                                if line1_length * 1.01 > (int1_dist_1 + int1_dist_2) and line2_length * 1.01 > (int2_dist_1 + int2_dist_2):
                                    intsct_counter += 1
                                    # now check which points are in the polygon and create segments accordingly
                                    if is_point_in_polygon_xy(l1[0], rec2):
                                        final_points.add(l1[0])
                                        segments.append([l1[0], intersection_pt])
                                    if is_point_in_polygon_xy(l1[1], rec2):
                                        final_points.add(l1[1])
                                        segments.append([l1[1], intersection_pt])
                                    if is_point_in_polygon_xy(l2[0], rec1):
                                        final_points.add(l2[0])
                                        segments.append([l2[0], intersection_pt])
                                    if is_point_in_polygon_xy(l2[1], rec1):
                                        final_points.add(l2[1])
                                        segments.append([l2[1], intersection_pt])

                    surface_points = set()

                    def segment_cleaner(segs):
                        deletes = set()
                        for j, seg in enumerate(segs):
                            # check whether the length might be 0
                            difference = 0
                            for i in range(2):
                                difference += abs(seg[0][i] - seg[1][i])
                            if difference == 0.0:
                                deletes.add(j)
                            else:
                                # now check whether they might be just duplicates
                                for k in range(j + 1, len(segs)):
                                    other_seg = segs[k]
                                    if (seg[0] == other_seg[0] and seg[1] == other_seg[1]) or (seg[1] == other_seg[0]
                                                                                               and seg[0] == other_seg[1]):
                                        deletes.add(k)
                        for index in sorted(deletes, reverse=True):
                            del segs[index]
                        return segs

                    for segment in segments:
                        for pt in segment:
                            new_point = (round(pt[0], 3), round(pt[1], 3), round(brd2.tool_frame[0][2], 3))
                            surface_points.add(new_point)

                    if len(segments) > 4:
                        segments = segment_cleaner(segments)

                    if 2 <= len(segments):
                        # three cases:
                        # 1) Vectors parallel and same direction
                        # 2) Vectors parallel but opposite directions
                        # 3) Vectors perpendicular originating in the same point

                        corner_pts_temp = []
                        for corner_pt in surface_points:
                            corner_pts_temp.append(corner_pt)

                        if len(corner_pts_temp) == 2:
                            return None

                        # just in case 3 to add the last point
                        if len(corner_pts_temp) == 3:
                            v01 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[1])
                            v02 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[2])
                            v12 = Vector.from_start_end(corner_pts_temp[1], corner_pts_temp[2])
                            v01_blank = [v01[0], v01[1], v01[2]]
                            v02_blank = [v02[0], v02[1], v02[2]]
                            v12_blank = [v12[0], v12[1], v12[2]]
                            pt0 = [corner_pts_temp[0][0], corner_pts_temp[0][1], corner_pts_temp[0][2]]
                            pt1 = [corner_pts_temp[1][0], corner_pts_temp[1][1], corner_pts_temp[1][2]]
                            pt2 = [corner_pts_temp[2][0], corner_pts_temp[2][1], corner_pts_temp[2][2]]

                            last_pt = [0, 0, 0]
                            if max(v01.length, v02.length, v12.length) == v01.length:
                                for i in range(3):
                                    last_pt[i] = pt2[i] - v02_blank[i] - v12_blank[i]
                            elif max(v01.length, v02.length, v12.length) == v02.length:
                                for i in range(3):
                                    last_pt[i] = pt1[i] - v01_blank[i] + v12_blank[i]
                            else:
                                for i in range(3):
                                    last_pt[i] = pt0[i] + v01_blank[i] + v02_blank[i]

                            last_pt_final = (last_pt[0], last_pt[1], last_pt[2])
                            corner_pts_temp.append(last_pt_final)

                        vec01 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[1])
                        vec02 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[2])
                        vec03 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[3])

                        # make sure that the second point is right
                        if vec01.length > max(vec02.length, vec03.length):
                            a = corner_pts_temp[1]
                            corner_pts_temp[1] = corner_pts_temp[2]
                            corner_pts_temp[2] = a

                        # now make sure that the third point is right
                        vec12 = Vector.from_start_end(corner_pts_temp[1], corner_pts_temp[2])
                        vec13 = Vector.from_start_end(corner_pts_temp[1], corner_pts_temp[3])
                        if vec13.length < vec12.length:
                            a = corner_pts_temp[2]
                            corner_pts_temp[2] = corner_pts_temp[3]
                            corner_pts_temp[3] = a

                        # now make sure that the points are clockwise
                        vec01 = Vector.from_start_end(corner_pts_temp[0], corner_pts_temp[1])
                        vec12 = Vector.from_start_end(corner_pts_temp[1], corner_pts_temp[2])

                        # check if it's clockwise
                        if Vector.angle_signed(vec01, vec12, Vector(0, 0, 1)) > 0:
                            return Polygon([corner_pts_temp[0], corner_pts_temp[1], corner_pts_temp[2], corner_pts_temp[3]])
                        else:
                            return Polygon([corner_pts_temp[0], corner_pts_temp[3], corner_pts_temp[2], corner_pts_temp[1]])
                    # no intersection
                    else:
                        return None

            def line_creator(pt_a, vec, length):
                end_pt = pt_a + scale_vector(vec, length / 2)
                start_pt = pt_a + scale_vector(vec, length / -2)
                return start_pt, end_pt

            # creates a snaking glue path
            def gluepath_creator(int_surf, path_width, board):
                def interval_checker(dimension):
                    underflow = dimension % path_width
                    if underflow > 0.02:
                        no_paths = dimension // path_width + 1
                        new_path_width = dimension / no_paths
                        return new_path_width
                    else:
                        return path_width

                finished = False
                while not finished:
                    finished = True
                    wid_gap = int_surf[1] - int_surf[0]
                    wid_vec = Vector(wid_gap[0], wid_gap[1], wid_gap[2])
                    wid = wid_vec.length
                    wid_vec.unitize()
                    len_gap = int_surf[2] - int_surf[1]
                    len_vec = Vector(len_gap[0], len_gap[1], len_gap[2])
                    len = len_vec.length
                    len_vec.unitize()
                    wid_path = interval_checker(wid)
                    len_path = interval_checker(len)
                    path_dims = [wid_path, len_path]
                    path_points = []
                    iteration = 0
                    path_unfinished = True
                    current_pt = int_surf[0] + scale_vector(wid_vec, wid_path / 2) + scale_vector(len_vec, len_path / 2)
                    current_vec = len_vec.unitized()
                    len_left = len - len_path
                    wid_left = wid - wid_path
                    dims_left = [len_left, wid_left]
                    path_points.append(current_pt)
                    R = Rotation.from_axis_and_angle([0, 0, 1], -math.pi / 2)
                    while path_unfinished:
                        current_index = iteration % 2
                        current_dim = dims_left[current_index]
                        if iteration > 2:
                            current_dim -= path_dims[current_index]
                            dims_left[current_index] = current_dim

                        if current_dim < path_width * 0.95:
                            break
                        current_pt = current_pt + scale_vector(current_vec, current_dim)
                        path_points.append(current_pt)
                        current_vec.transform(R)
                        current_vec.unitize()
                        iteration += 1
                        if not is_point_in_polygon_xy(current_pt, int_surf):
                            print("Warning: Gluepath point not in polygon. We try to fix it but in case better check the gluepaths to be safe.")
                            print(board.global_count)
                            print("\n")
                            # semi-clean solution: If the points are anti-clockwise then change the polygon and try it again
                            new_polygon = Polygon([int_surf[0], int_surf[3], int_surf[2], int_surf[1]])
                            int_surf = new_polygon
                            finished = False
                            break
                    if finished:
                        return path_points
                    else:
                        continue

            # creates a point or line
            def gluepoint_creator(int_surf, my_board, board2):
                # see if we should get a point or a line
                if int_surf.length > (my_board.width + board2.width) * 2.1:
                    if int_surf.lines[0].length < int_surf.lines[1].length:
                        # first line is the short side
                        return [int_surf.lines[0].midpoint, int_surf.lines[2].midpoint]
                    else:
                        # first line is on the long side
                        return [int_surf.lines[1].midpoint, int_surf.lines[3].midpoint]
                else:
                    # if it's just a point then it's pretty easy
                    my_glue_point = int_surf.centroid
                    return [my_glue_point]

            def relative_gluepoints_converter(my_board, bending=None, gluestation=True):

                def bending_calculator(my_vector, my_brd, bend):
                    # make sure the user described the bending behaviour correctly
                    if len(bend) != 3:
                        print("Warning: Wrong bending input, not taken into consideration")
                        return 0

                    tool_length = bend[0]
                    excentricity = my_vector.length - tool_length

                    # if it's on the tool then we have no bending at all
                    if excentricity < 0:
                        return 0

                    # should be somewhere around 0.2-0.8
                    bending_factor = bend[1]
                    stiffness = ((my_brd.height * 100) ** 1.5) * ((my_brd.width * 100) ** 0.5)
                    downbending = excentricity / stiffness * bending_factor
                    return downbending

                for path in my_board.glue_paths:
                    path_frames = []
                    for point in path:
                        absolute_glue_frame = Frame(point, my_board.tool_frame[1], my_board.tool_frame[2])
                        if not gluestation:
                            path_frames.append(absolute_glue_frame)
                            continue

                        gluepoint_toolpoint_vector = Vector.from_start_end(my_board.tool_frame[0], absolute_glue_frame[0])
                        board_vector = Vector(my_board.tool_frame[1][0], my_board.tool_frame[1][1], my_board.tool_frame[1][2])
                        station_vector = Vector(glue_station_default_frame[1][0], glue_station_default_frame[1][1], glue_station_default_frame[1][2])

                        angle_station_board = board_vector.angle(station_vector)
                        glue_pt = Point(absolute_glue_frame[0][0], absolute_glue_frame[0][1], absolute_glue_frame[0][2])
                        tool_pt = Point(my_board.tool_frame[0][0], my_board.tool_frame[0][1], my_board.tool_frame[0][2])
                        glue_point_rotated_raw = rotate_points([glue_pt], angle_station_board, Vector(0, 0, 1), tool_pt)
                        glue_point_rotated = Point(glue_point_rotated_raw[0][0], glue_point_rotated_raw[0][1], glue_point_rotated_raw[0][2])

                        station_gluepoint_vector = Vector.from_start_end(tool_pt, glue_point_rotated)
                        station_gluepoint = glue_station_default_frame[0] + station_gluepoint_vector

                        # consider the bending of the timber board, don't do it with a longer path unless you specify so in the bending behaviour
                        if bending:
                            if (len(path) == 1 or bending[2] == True):
                                z_deviation = bending_calculator(station_gluepoint_vector, my_board, bending)
                                station_gluepoint[2] += z_deviation

                        relative_glue_frame = Frame(station_gluepoint, glue_station_default_frame[1],
                                                    glue_station_default_frame[2])

                        path_frames.append(relative_glue_frame)

                    my_board.final_glue_frames.append(path_frames)

                # some primitive sorting
                if not sorting_flipped:
                    if my_board.final_glue_frames[0][0][0][sorting_direction] > my_board.final_glue_frames[-1][0][0][sorting_direction]:
                        my_board.final_glue_frames.reverse()
                else:
                    if my_board.final_glue_frames[0][0][0][sorting_direction] > my_board.final_glue_frames[-1][0][0][sorting_direction]:
                        my_board.final_glue_frames.reverse()

            def create_assembly_path(my_board):
                def safety_frame_creator(my_frame, safety_dist=safety_distance):
                    my_frame_z_vector = my_frame.zaxis
                    my_frame_z_vector.scale(safety_dist * -1)
                    safety_frame = Frame(Point(my_frame[0][0] + my_frame_z_vector[0], my_frame[0][1] + my_frame_z_vector[1], my_frame[0][2] + my_frame_z_vector[2]),
                                         my_frame[1], my_frame[2])
                    return safety_frame

                def flipped_safety_frame(my_frame):
                    # move the whole thing up
                    my_frame_z_vector = my_frame.zaxis
                    my_frame_z_vector.scale(flipped_dropframe_distance * -1)
                    flipped_tool_point = Point(my_frame[0][0] + my_frame_z_vector[0], my_frame[0][1] + my_frame_z_vector[1], my_frame[0][2] + my_frame_z_vector[2])

                    # flip it now
                    my_frame_flipped_y_vector = my_frame[2].scaled(-1)

                    flipped_frame = Frame(flipped_tool_point, my_frame[1], my_frame_flipped_y_vector)
                    return flipped_frame

                # between the gluepoints we probably don't need nearly as much safety distance
                if not safety_distance_gluepoints:
                    safety_distance_glue = safety_distance
                else:
                    safety_distance_glue = safety_distance_gluepoints

                # first head to the stack
                safety_frame_stack = safety_frame_creator(my_board.stack_pick_frame)
                path = [(safety_frame_stack, ["free"]), (my_board.stack_pick_frame, ["cart"]), (safety_frame_stack, ["cart"])]
                safety_frame_drop_motion_type = "free"
                # now head to the glue part
                if len(my_board.final_glue_frames) > 0:

                    path.append((safety_frame_creator(my_board.final_glue_frames[0][0]), ["free"]))
                    for i, gluepath in enumerate(my_board.final_glue_frames):
                        if i > 0:
                            path.append((safety_frame_creator(gluepath[0], safety_distance_glue), ["cart"]))
                        for point in gluepath:
                            path.append((point, ["cart"]))
                        if i < len(my_board.final_glue_frames) - 1:
                            path.append((safety_frame_creator(gluepath[-1], safety_distance_glue), ["cart"]))
                    path.append((safety_frame_creator(my_board.final_glue_frames[-1][-1]), ["cart"]))

                    # now head to the drop zone
                    if flipped_dropframe_distance and glue_station_default_frame:
                        safety_frame_drop_flipped = flipped_safety_frame(my_board.tool_frame)
                        path.append((safety_frame_drop_flipped, ["cart"]))
                    safety_frame_drop_motion_type = "cart"
                safety_frame_drop = safety_frame_creator(my_board.tool_frame)
                path.append((safety_frame_drop, [safety_frame_drop_motion_type]))
                path.append((my_board.tool_frame, ["cart"]))
                path.append((safety_frame_drop, ["cart"]))
                return path

            # actual procedure
            for layer_number in range(1, self.layer_no):
                for brd in self.elements():
                    board = brd[1]
                    if board.layer < layer_number or dryrun:
                        # exclude the first layer of boards
                        board.assembly_path = create_assembly_path(board)
                        continue
                    elif board.layer > layer_number:
                        break
                    else:
                        for i, other_brd in enumerate(self.elements()):
                            other_board = other_brd[1]
                            if other_board.layer < layer_number - 1:
                                continue
                            elif other_board.layer > layer_number - 1:
                                break
                            else:
                                my_glue_surface = board_intersection(board, other_board)
                                if my_glue_surface is None:
                                    continue
                                board.glue_surfaces.append(my_glue_surface)
                                if snake:
                                    board.glue_paths.append(gluepath_creator(my_glue_surface, gluepath_width, board))
                                else:
                                    board.glue_paths.append(gluepoint_creator(my_glue_surface, board, other_board))
                                self.network.edge[board.global_count][i] = self.network.node[other_board.global_count]

                    if glue_station_default_frame:
                        relative_gluepoints_converter(board, bending_behaviour, gluestation=True)
                    else:
                        relative_gluepoints_converter(board, gluestation=False)
                    if safety_distance:
                        board.assembly_path = create_assembly_path(board)
                    else:
                        print("Warning: No assembly path created because no Safety Distance was given")
            return self

        def stack_creator(max_height):
            instances = self.component_instances_export()
            stacks = []
            bottom_row_pickup_z = bottom_pickup
            first_column_position = col_pickup
            # since leftover pieces are often put on top
            max_height -= 1
            for profile in instances:
                profile_width = profile[0][0][0]
                profile_height = profile[0][0][1]
                for i in range(1, len(profile)):
                    profile_length = profile[i][0]
                    no_elements = profile[i][1]
                    no_columns = no_elements // max_height
                    if no_elements % max_height != 0:
                        no_columns += 1
                    print("no_elements: {}, no_cols: {}, max_height: {}".format(no_elements, no_columns, max_height))
                    overflow = max_height * no_columns - no_elements
                    if overflow > no_columns:
                        no_rows = max_height - overflow // no_columns
                        first_board_picked = overflow % no_columns
                    elif overflow > 0 and no_columns > 1:
                        # if columns * rows does not lead to no_elements leave away the first few pieces in the top row
                        first_board_picked = overflow
                        no_rows = max_height
                    elif overflow > 0:
                        # if there are not even enough elements for one regular column
                        no_rows = max_height - overflow
                        no_columns = 1
                        first_board_picked = 0
                    else:
                        # if it just works out
                        no_rows = max_height
                        first_board_picked = 0

                    # in case one wants to make life easier and just fill up stack to complete rows
                    if not full_efficiency:
                        first_board_picked = 0
                    new_stack = {"profile_width": profile_width, "profile_height": profile_height,
                                 "profile_length": profile_length, "no_elements": no_elements,
                                 "no_rows": no_rows, "no_columns": no_columns, "next_board": first_board_picked,
                                 "first_column_position": first_column_position + profile_width / 2,
                                 "first_row_position": bottom_row_pickup_z + profile_height}
                    # print(new_stack)
                    stacks.append(new_stack)
                    first_column_position += no_columns * profile_width + (no_columns // max_width) * distance_within_stack \
                                             + distance_between_stacks

            # print("")
            # now that the stacks have been created, we can assign boards to them

            # stupid function to add vectors and points because it doesn't work somehow
            def vector_point_addition(items):
                final_points = []
                for i in range(3):
                    current_value = 0
                    for item in items:
                        current_value += item[i]
                    final_points.append(current_value)
                return Point(final_points[0], final_points[1], final_points[2])

            for my_brd in self.elements():
                my_board = my_brd[1]

                for stack_id, my_stack in enumerate(stacks):
                    if (my_board.length == my_stack["profile_length"] and my_board.width == my_stack["profile_width"] and
                        my_board.height == my_stack["profile_height"]):
                        # print(my_stack)
                        no_in_stack = my_stack["next_board"]
                        col = no_in_stack % my_stack["no_columns"]
                        row = my_stack["no_rows"] - no_in_stack // my_stack["no_columns"]
                        my_stack["next_board"] += 1

                        pick_x = my_stack["profile_length"] / 2
                        pick_y = my_stack["first_column_position"] + col * my_stack["profile_width"] + col // max_width * distance_within_stack
                        pick_z = row * my_stack["profile_height"] + my_stack["profile_height"]
                        stack_origin_point = stack_origin_frame[0]
                        print(stack_origin_point)
                        print(stack_origin_frame)
                        print("\n")
                        pick_point = vector_point_addition([stack_origin_point, stack_origin_frame[1] * pick_x,
                                                            stack_origin_frame[2] * pick_y, Vector(0, 0, 1) * pick_z])
                        #pick_vector = Vector(stack_origin_frame[1]) * pick_x + stack_origin_frame[2] * pick_y + Vector(0, 0, 1) * pick_z
                        #pick_point = (stack_origin_point + stack_origin_frame[1] * pick_x + stack_origin_frame[2] * pick_y +
                         #             Vector(0, 0, 1) * pick_z)
                        #pick_point = Point(stack_origin_point[0] + pick_vector[0], stack_origin_point[1] + pick_vector[1], stack_origin_point[2] + pick_vector[2])
                        stack_pick_fram = Frame(pick_point, stack_origin_frame[1], stack_origin_frame[2])
                        stack_center_fram = Frame(Point(pick_point[0], pick_point[1], pick_point[2] - my_stack["profile_height"] / 2),
                                                  stack_origin_frame[1], stack_origin_frame[2])
                        my_board.stack_pick_frame = stack_pick_fram
                        my_board.stack_center_frame = stack_center_fram
                        my_board.stack_index = stack_id
                        # print("stack_id:{}, no_in_stack:{}, column:{}, row: {}".format(stack_id, no_in_stack, col, row))

        stack_creator(max_hi)
        gluepoints()
        print("hello")



origin_point = Point(0, 0, 0)
origin_vector_primary = Vector(0, 1, 0)
origin_vector_secondary = Vector(1, 0, 0)
origin_frame = Frame(origin_point, origin_vector_primary, origin_vector_secondary)
my_floorslab = TCHAssembly(5, 0.01, 3.0, 2.5, False, 0.06, 0.04, 0.06, 0.04, 0.06, 0.04, 0.1, 0.5, 2, 0.2, 1.2, True, origin_frame, True, True, 0.04, 0.04)

my_geometry = my_floorslab.floorslab_geometry_creation()
my_floorslab.assembly_creation(origin_frame, safety_distance=0.4)
