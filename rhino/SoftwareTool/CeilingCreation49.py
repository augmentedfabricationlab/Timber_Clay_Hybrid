import math
import copy
import operator

from compas.geometry import Point, Box, Frame, Vector, scale_vector, normalize_vector
from compas.datastructures import Mesh
from compas.geometry import Line
from compas.geometry import intersection_line_line_xy, is_intersection_line_line_xy
from compas.geometry import distance_point_point
from compas_ghpython.artists import MeshArtist


class Floorslabsystem:
    def __init__(self, no_layers, min_gap, len_prim, len_sec, omni, prim_wid_out, prim_hi_out, prim_wid_in, prim_hi_in,
                 sec_wi, sec_hi, prim_intv, prim_fal, prim_dedens, sec_intv, sec_intv_dev, orgn_fram, skip):

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
        if self.omnidirectional:
            self.primary_falloff = prim_fal
            self.primary_dedensification = prim_dedens
        else:
            self.primary_falloff = 0.0
            self.primary_dedensification = 0
        self.secondary_interval = sec_intv
        self.secondary_interval_development = sec_intv_dev
        self.skipping = skip
        self.primary_direction = 0
        self.secondary_direction = 1
        self.origin_fr = orgn_fram
        self.origin_pt = orgn_fram[0]
        self.prim_dir = orgn_fram[1]
        self.sec_dir = orgn_fram[2]
        self.sec_fr = Frame(self.origin_pt, self.sec_dir, self.prim_dir)
        self.timberboards = []
        self.setup_done = False

        self.prim_vert_sup = False
        self.sec_vert_sup = False
        self.vert_sup_lengths = None
        self.vert_sup_gap_tolerance = 0.2
        self.vert_sup_gap_min = 0.5

        # ADVANCED PARAMETERS
        self.advanced_setup = False

    # Advanced geometry setup
    def advanced_floorslab_setup(self, prim_in_sup_length, prim_out_sup_length, prim_in_sup_gap_min=0.0, prim_in_sup_layers=None,
                               prim_in_sup_dist_edge_min=0.0, prim_in_sup_dist_edge_max=100000000, prim_out_sup_gap_min=0.0,
                               prim_out_sup_layers=None, prim_out_sup_dist_edge_min=0.0, prim_out_sup_dist_edge_max=100000000):

        if prim_out_sup_layers is None:
            prim_out_sup_layers = []
        if prim_in_sup_layers is None:
            prim_in_sup_layers = []
        self.advanced_setup = True

        # Inside Supporters
        self.primary_inside_support_gap_min = prim_in_sup_gap_min
        self.primary_inside_support_length = prim_in_sup_length
        self.primary_inside_support_layers = prim_in_sup_layers
        self.primary_inside_support_distance_to_edge_min = prim_in_sup_dist_edge_min
        self.primary_inside_support_distance_to_edge_max = prim_in_sup_dist_edge_max
        self.primary_inside_support_dimensions = copy.deepcopy(self.primary_board_inside_dimensions)
        self.primary_inside_support_dimensions[2] = self.primary_inside_support_length

        # Outside Supporters
        self.primary_outside_support_gap_min = prim_out_sup_gap_min
        self.primary_outside_support_length = prim_out_sup_length
        self.primary_outside_support_layers = prim_out_sup_layers
        self.primary_outside_support_distance_to_edge_min = prim_out_sup_dist_edge_min
        self.primary_outside_support_distance_to_edge_max = prim_out_sup_dist_edge_max
        self.primary_outside_support_dimensions = copy.deepcopy(self.primary_board_outside_dimensions)
        self.primary_outside_support_dimensions[2] = self.primary_outside_support_length

    def vert_sup_setup(self, prim_board_sup=False, sec_board_sup=False, available_lengths=None, gap_tolerance=0.2, vert_gap_min=5.0):
        self.prim_vert_sup = prim_board_sup
        self.sec_vert_sup = sec_board_sup
        self.vert_sup_lengths = available_lengths
        self.vert_sup_gap_tolerance = gap_tolerance
        self.vert_sup_gap_min = vert_gap_min

    # makes sure that the program doesn't crash when the user/algorithm enters some nonsense
    def input_check(self):
        input_validity = True
        # check for too much dedensification
        if self.primary_dedensification > 0:
            if self.primary_falloff/self.primary_interval - self.primary_dedensification < 1:
                input_validity = False
                print("Too much dedensification")

        # check whether the interval is realistic
        if self.primary_dedensification == 0:
            interval_intolerance = (self.secondary_length-self.primary_board_width_outside) % self.primary_interval
            no_intervals = (self.secondary_length-self.primary_board_width_outside) // self.primary_interval
            if interval_intolerance > 0.1:
                interval_intolerance /= (no_intervals-1)
                self.primary_interval += interval_intolerance


        return input_validity

    # calculates the basic centre lines
    # basically just a list of numbers, the rest comes later
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
        # not sure what this was supposed to do precisely, didn't work in this instance
        inacc = (primary_graph_centre_line_x - primary_graph_centre_minimum_x) % self.primary_interval
        target = primary_graph_centre_minimum_x + inacc

        centre_ratio = (target - primary_graph_centre_line_x) / -primary_graph_centre_line_x

        # now start from scratch now with a different ratio
        primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio * primary_graph_centre_line_x
        primary_graph_centre_maximum_x = primary_graph_centre_line_x + centre_ratio * primary_graph_centre_line_x

        print(self.primary_interval)
        print((self.secondary_length-self.primary_board_width_outside)/self.primary_interval)

        # +.1 is a bit of a dirty fix against float inaccuracies
        no_elements = ((primary_graph_centre_line_x - primary_graph_centre_minimum_x) + 0.1) // self.primary_interval

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
        # +1 is an unelegant solution that makes sure the highest element is still respected
        for i in range(0, int(secondary_graph_y_maximum) + 1, 1000):
            # omnidirectional case
            if self.primary_dedensification > 0:
                result = graph_function(i)
                if result == 2:
                    break
            # monodirectional case
            else:
                result = self.primary_board_width_outside/2 + (i/1000)*self.primary_interval
            primary_span_grid[0].append(result)
        # dirty solution because otherwise the last board would have been skipped
        if self.primary_dedensification == 0:
            primary_span_grid[0].append(self.secondary_length - self.primary_board_width_outside/2)


        # now create also the alternative lines in the gaps of the grid
        for i in range(500, int(secondary_graph_y_maximum) + 1, 1000):
            result = graph_function(i)
            if result == 2:
                break
            else:
                primary_span_grid[1].append(result)

        # now a safety thing to make sure that there is a position at the ends
        if primary_span_grid[0][0] > self.primary_board_width_outside/1.9:
            print("Error: No board on the sides")

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

        no_secondary_span_elements = self.primary_length // self.secondary_interval
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
            secondary_span_grid[j].insert(0, self.secondary_board_width / 2)
            secondary_span_grid[j].append(self.primary_length - self.secondary_board_width / 2)

        secondary_span_grid[0].sort()
        return primary_span_grid, secondary_span_grid

    # adapts the advanced boards to meaningful lengths
    def __advanced_length_correction(self):
        # inside shear supports
        print(self.primary_inside_support_length)
        position = 0
        while self.primary_inside_support_length > self.floorslab_grids[1][0][position]:
            position += 1
        # //1 just to get a nicer number
        self.primary_inside_support_length = (self.floorslab_grids[1][0][position] + self.secondary_board_width/2) // 1 + 1
        self.primary_inside_support_dimensions[2] = self.primary_inside_support_length

        # outside momentum supports
        outside_grid = self.floorslab_grids[1][0]
        outside_centre_id = len(outside_grid) // 2
        position = 1
        while outside_grid[outside_centre_id + position] - outside_grid[outside_centre_id - position] < self.primary_outside_support_length:
            position += 1
        # //1 just to get a nicer number
        self.primary_outside_support_length = (outside_grid[outside_centre_id + position] - outside_grid[outside_centre_id - position]) // 1 + 1
        self.primary_outside_support_dimensions[2] = self.primary_outside_support_length
        print(self.primary_outside_support_length)

    # default setup that collects the relevant functions
    def floorslab_creation(self):
        print(self.primary_interval)
        if self.input_check():
            self.floorslab_grids = []
            self.timberboards = []
            self.floorslab_grids = self.__grid_creation()
            if self.advanced_setup:
                self.__advanced_length_correction()
            self.__board_data_setup(self.advanced_setup)
            if self.prim_vert_sup or self.sec_vert_sup:
                self.vert_load_supports()
            self.board_geometry_setup()
            self.gluepoints()
            self.setup_done = True
        else:
            return 1

    def vert_load_supports(self):
        # first check space between the primary boards
        if self.vert_sup_lengths is not None:
            self.vert_sup_lengths.sort()
        global_counter = self.timberboards[-1][-1].index + 1
        for l, board_layer in enumerate(self.timberboards):
            if (l % 2 == 0 and not self.prim_vert_sup) or (l % 2 == 1 and not self.sec_vert_sup):
                continue
            layer_counter = board_layer[-1].no_in_layer + 1
            # reads the general board length from the first entry
            layer_length = board_layer[0].length
            gap = 1
            i = 0
            height = board_layer[0].height
            layer_width = board_layer[0].width
            while i < len(board_layer)-1:
                width = None
                # eliminate potential supporters that only exist in the middle
                current_board = board_layer[i]
                # make sure you don't catch the ones in the centre

                ##########################
                # HERE IS AN ISSUE WITH THE SHEARS IN ADVANCED SETUP
                ##########################

                # if it's a centre thing itself (unlikely case, though)
                if current_board.length < layer_length and current_board.location == "centre":
                    i += 1
                    continue
                next_board = board_layer[i+gap]
                if current_board.length < layer_length and current_board.location == "centre":
                    gap += 1
                    continue
                current_gap = next_board.grid_position - current_board.grid_position - layer_width
                # reset, we are doing that already now so we don't have to do it twice
                i += gap
                gap = 1
                if current_gap > self.vert_sup_gap_min:
                    centre_position = (current_board.grid_position + next_board.grid_position) / 2
                    # be aware that width and length are kind of switched in this case
                    # now we must find the width
                    ideal_width = current_gap - self.vert_sup_gap_tolerance
                    if self.vert_sup_lengths is None:
                        width = ideal_width
                    else:
                        for j, entry in enumerate(self.vert_sup_lengths):
                            if entry > ideal_width and j > 0:
                                width = self.vert_sup_lengths[j-1]
                            elif j > 0:
                                continue
                            else:
                                print("Warning: No suitable length available")
                                width = ideal_width
                        if width is None:
                            width = self.vert_sup_lengths[-1]

                    # now the length
                    if l%2 == 1:
                        length = self.primary_board_width_outside
                    else:
                        length = self.secondary_board_width

                    self.timberboards[l].append(self.Timberboard(l, global_counter, layer_counter, [width, length, height], board_layer[0].z_drop,
                                                         l % 2, abs(l % 2 - 1), centre_position, "low", True))
                    layer_counter += 1
                    global_counter += 1
                    self.timberboards[l].append(self.Timberboard(l, global_counter, layer_counter, [width, length, height], board_layer[0].z_drop,
                                                         l % 2, abs(l % 2 - 1), centre_position, "high", True))
                    layer_counter += 1
                    global_counter += 1
                else:
                    continue
            board_layer.sort()
        return 0

    # weight calculation
    def weight_calculator(self, protective_clay_height = 5.0, density_timber = 460, density_clay = 1900):
        # safety loop in the beginning
        if not self.setup_done:
            print("Error: Setup not completed yet")
            return 1

        # actual program
        unit_factor = 1000000
        total_height = protective_clay_height
        timber_volume = 0.0
        clay_volume = 0.0
        for board_layer in self.timberboards:
            total_height += board_layer[0].height
            for board in board_layer:
                timber_volume += board.width*board.height*board.length
        total_volume = (total_height * primary_length * secondary_length)/unit_factor
        timber_volume /= unit_factor
        clay_volume = total_volume - timber_volume
        total_weight = clay_volume * density_clay + timber_volume * density_timber
        relative_weight = total_weight/((primary_length/100)*(secondary_length/100))

        print("[Total Weight, Weight/sqm, Total Timber Volume (m3), Total Clay Volume (m3)")
        return [total_weight, relative_weight, timber_volume, clay_volume]

    # subclass: the individual timber boards
    class Timberboard:
        def __init__(self, board_layer, identification, board_no_in_layer, board_dimensions,
                     z_value_toppoint, length_dir, width_dir, grid_pos, position="centre", sup = False):
            self.index = identification
            self.layer = board_layer
            self.no_in_layer = board_no_in_layer
            self.dimensions = board_dimensions
            self.width = self.dimensions[0]
            self.height = self.dimensions[1]
            self.length = self.dimensions[2]
            self.drop_point = Point(0, 0, 0)
            self.centre_point = Point(0, 0, 0)
            self.z_drop = z_value_toppoint
            self.length_direction = length_dir
            self.width_direction = width_dir
            self.glue_givers = []
            self.glue_receivers = []
            self.receiving_neighbours = []
            self.giving_neighbours = []
            self.grid_position = grid_pos
            self.location = position
            self.supporter = sup

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

        def __lt__(self, other):
            return self.grid_position < other.grid_position

    # creates the timber board instances and equips them with dimensions and z-location
    def __board_data_setup(self, advanced):
        z_value = 0
        global_counter = 0
        for layer in range(self.layer_no):
            layer_counter = 0
            self.timberboards.append([])
            # outside layer, primary_span
            if layer == 0 or layer == self.layer_no - 1:
                boardheight = self.primary_board_height_outside
                z_value += boardheight
                for i in range(len(self.floorslab_grids[0][0])):
                    self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter, self.primary_board_outside_dimensions,
                                                                     z_value, self.primary_direction, self.secondary_direction,
                                                                     self.floorslab_grids[0][0][i]))
                    global_counter += 1
                    layer_counter += 1

                # inserts the momentum support pieces if it's wished
                if advanced and self.primary_outside_support_length > 0:
                    for i in range(len(self.floorslab_grids[0][1])):

                        if (self.timberboards[layer][i + 1].grid_position - self.timberboards[layer][i].grid_position >
                            self.primary_outside_support_gap_min and
                            (self.timberboards[layer][i + 1].grid_position - self.timberboards[layer][i].grid_position) / 2 -
                            self.primary_board_width_outside > self.gap_min and
                            self.floorslab_grids[0][1][i] > self.primary_outside_support_distance_to_edge_min and
                            self.secondary_length - self.floorslab_grids[0][1][i] > self.primary_outside_support_distance_to_edge_min and
                            self.floorslab_grids[0][1][i] < self.primary_outside_support_distance_to_edge_max and
                            self.secondary_length - self.floorslab_grids[0][1][i] < self.primary_outside_support_distance_to_edge_max and
                            (self.primary_outside_support_layers == [] or layer in self.primary_outside_support_layers)):

                            self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter,
                                                                             self.primary_outside_support_dimensions,
                                                                             z_value, self.primary_direction,
                                                                             self.secondary_direction, self.floorslab_grids[0][1][i]))

                            global_counter += 1
                            layer_counter += 1

            # inside layer, primary span
            elif layer % 2 == 0:
                boardheight = self.primary_board_height_inside
                z_value += boardheight
                # no dedensification on the inside
                if not self.skipping:
                    for i in range(len(self.floorslab_grids[0][0])):
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, i, self.primary_board_inside_dimensions,
                                                                         z_value, self.primary_direction, self.secondary_direction,
                                                                         self.floorslab_grids[0][0][i]))
                        global_counter += 1

                # dedensification on the inside
                else:
                    # not sure about the -1
                    max_i = 0
                    for i in range(0, len(self.floorslab_grids[0][0]), 2):
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter, self.primary_board_inside_dimensions,
                                                                         z_value, self.primary_direction, self.secondary_direction,
                                                                         self.floorslab_grids[0][0][i]))
                        global_counter += 1
                        layer_counter += 1

                    # last element??
                    if len(self.floorslab_grids[0][0]) % 2 != 1:
                        self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter, self.primary_board_inside_dimensions,
                                                                         z_value, self.primary_direction,
                                                                         self.secondary_direction, self.floorslab_grids[0][0][-1]))
                        global_counter += 1
                        layer_counter += 1

                    # inserts the shear support pieces if wished
                    if advanced and self.primary_outside_support_length > 0:
                        for i in range(1, len(self.floorslab_grids[0][0]), 2):
                            # now check whether there should really be a board at that position
                            if (self.timberboards[layer][i//2 + 1].grid_position - self.timberboards[layer][i//2].grid_position >
                                self.primary_inside_support_gap_min and
                                (self.timberboards[layer][i//2 + 1].grid_position - self.timberboards[layer][i//2].grid_position)/2 -
                                self.primary_board_width_inside > self.gap_min and
                                self.floorslab_grids[0][1][i] > self.primary_inside_support_distance_to_edge_min and
                                self.secondary_length - self.floorslab_grids[0][1][i] > self.primary_inside_support_distance_to_edge_min and
                                self.floorslab_grids[0][1][i] < self.primary_inside_support_distance_to_edge_max and
                                self.secondary_length - self.floorslab_grids[0][1][i] < self.primary_inside_support_distance_to_edge_max and
                                (self.primary_inside_support_layers == [] or layer in self.primary_inside_support_layers)):

                                self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter,
                                                                                 self.primary_inside_support_dimensions,
                                                                                 z_value, self.primary_direction,
                                                                                 self.secondary_direction, self.floorslab_grids[0][0][i],
                                                                                 "low"))
                                layer_counter += 1
                                self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter,
                                                                                 self.primary_inside_support_dimensions,
                                                                                 z_value, self.primary_direction,
                                                                                 self.secondary_direction, self.floorslab_grids[0][0][i],
                                                                                 "high"))
                                global_counter += 2
                                layer_counter += 1
                            else:
                                continue

            # secondary span
            else:
                boardheight = self.secondary_board_height
                z_value += boardheight
                for i in range(0, len(self.floorslab_grids[1][0])):
                    self.timberboards[layer].append(self.Timberboard(layer, global_counter, layer_counter, self.secondary_board_dimensions,
                                                                     z_value, self.secondary_direction, self.primary_direction,
                                                                     self.floorslab_grids[1][0][i]))
                    global_counter += 1
                    layer_counter += 1

        for board_layer in self.timberboards:
            board_layer.sort()

    # the actual geometry setup
    def board_geometry_setup(self):
        for my_layer in range(self.layer_no):
            if my_layer % 2 == 0:
                my_frame = self.origin_fr
                my_grid = self.floorslab_grids[0][0]
            else:
                my_frame = self.sec_fr
                my_grid = self.floorslab_grids[1][0]
            my_dir1 = normalize_vector(my_frame[1])
            my_dir2 = normalize_vector(my_frame[2])

            # we have to separate i/board_code because of possible exceptions in the centre
            board_code = 0
            for my_board in self.timberboards[my_layer]:
                dist = my_board.grid_position
                # build the three vectors with which we later find he centre point
                # one advanced case
                if my_board.location == "high":
                    my_vec1 = scale_vector(my_dir1, self.primary_length - my_board.length / 2)
                elif my_board.location == "low":
                    my_vec1 = scale_vector(my_dir1, my_board.length / 2)
                else:
                    my_vec1 = scale_vector(my_dir1, self.timberboards[my_layer][0].length / 2)

                my_vec2 = scale_vector(my_dir2, dist)
                my_vec3 = Vector(0, 0, my_board.z_drop - my_board.height / 2)
                my_centre = self.origin_pt + my_vec1 + my_vec2 + my_vec3
                my_board.centre_point = my_centre
                my_board.drop_point = my_centre + Vector(0, 0, my_board.height / 2)
                my_board.length_vector = normalize_vector(my_vec1)
                my_board.width_vector = normalize_vector(my_vec2)
                my_board.box_update()
                board_code += 1

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
            if distance_point_point(pt1, int_pt) < len1 / 2 and \
                distance_point_point(pt2, int_pt) < len2 / 2:
                return int_pt
            else:
                return 0

        def line_creator(pt_a, vec, length):
            pt_b = pt_a + scale_vector(vec, length / 2)
            pt_a = pt_a - scale_vector(vec, length / 2)
            return pt_a, pt_b

        for layer_number in range(1, self.layer_no):
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

    def component_stack_export(self):
        stack = []
        for layer in self.timberboards:
            for board in layer:
                stack.append(board.dimensions)
        return stack

    def component_instances_export(self):
        global_boards = []
        for layer in self.timberboards:
            for board in layer:
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

    def grasshopper_draw(self):
        visualisations = []
        for woodLayer in self.timberboards:
            for board in woodLayer:
                board.box_update()
                my_box = board.box
                mesh_box = Mesh.from_shape(my_box)
                artist = MeshArtist(mesh_box)
                box_visualisation = artist.draw_mesh()
                visualisations.append(box_visualisation)
        return visualisations


# secondary_span_interval_development: 1 = constant, <1: denser in the centre, >1: denser on the edges
# operable range approximately 0.6/6

layer_no = 5
gap_min = 4.0
primary_length = 520.0
secondary_length = 901.0
omnidirectional = False
primary_board_width_outside = 6.0
primary_board_height_outside = 6.0
primary_board_width_inside = 6.0
primary_board_height_inside = 6.0
primary_board_outside_dimensions = [primary_board_width_outside, primary_board_height_outside, primary_length]
primary_board_inside_dimensions = [primary_board_width_inside, primary_board_height_inside, primary_length]
secondary_board_width = 4.0
secondary_board_height = 4.0
secondary_board_dimensions = [secondary_board_width, secondary_board_height, secondary_length]

primary_interval = 19.5
primary_falloff = 160.0
primary_dedensification = 6
secondary_interval = 80.0
secondary_interval_development = .89
skip_centrals = True

primary_direction = 0
secondary_direction = 1

origin_point = Point(0, 0, 0)
origin_vector_primary = Vector(0, 1, 0)
origin_vector_secondary = Vector(1, 0, 0)
origin_frame = Frame(origin_point, origin_vector_primary, origin_vector_secondary)

myFloorslab = Floorslabsystem(layer_no, gap_min, primary_length, secondary_length, omnidirectional,
                          primary_board_width_outside, primary_board_height_outside,
                          primary_board_width_inside, primary_board_height_inside,
                          secondary_board_width, secondary_board_height,
                          primary_interval, primary_falloff, primary_dedensification,
                          secondary_interval, secondary_interval_development, origin_frame, skip_centrals)

myFloorslab.advanced_floorslab_setup(150, 300)
myFloorslab.vert_sup_setup(True, True)
myFloorslab.floorslab_creation()
print(myFloorslab.weight_calculator())

# bei den Instances Exports muss man noch aufpassen
# print(myFloorslab.component_instances_export())


