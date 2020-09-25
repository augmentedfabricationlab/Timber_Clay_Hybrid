import math
import copy

from compas.geometry import Point, Box, Frame


def grid_creation(minimum_gap, primary_span_length, omnidirectional, primary_span_board_width_outside,
primary_span_board_height_outside, primary_span_board_width_inside, primary_span_board_height_inside,
secondary_span_length, secondary_span_board_width, secondary_span_board_height, primary_span_interval,
primary_falloff_length, primary_dedensification_intensity, secondary_span_interval,
secondary_span_interval_development):


    # in case the timber boards on the inside are the same size as usual
    if primary_span_board_height_inside < 0 or primary_span_board_width_inside < 0:
        primary_span_board_width_inside = primary_span_board_width_outside
        primary_span_board_height_inside = primary_span_board_height_outside

    # check for minimal distance
    # -.1 in order to be on the safe side
    minimum_gap -= .1
    if primary_span_board_height_outside < minimum_gap or primary_span_board_height_inside < minimum_gap or \
        (primary_span_interval - primary_span_board_width_outside) < minimum_gap or \
        (primary_span_interval - primary_span_board_width_inside) < minimum_gap:
        print("Too little gap")
        return 1

    # side_dedensification_intensity = how many elements do we kick out
    primary_dedensification_intensity *= 1000

    # define the room
    centre_ratio = (secondary_span_length - primary_falloff_length*2)/secondary_span_length

    primary_graph_centre_line_x = secondary_span_length/2

    # go into the borders
    # 1000 just so that we get nicer numbers
    primary_graph_default_ascent = 1000/primary_span_interval
    primary_graph_centre_line_y = (primary_graph_centre_line_x//primary_span_interval)*1000  - primary_dedensification_intensity
    primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio*primary_graph_centre_line_x

    # adjust the centre ratio in a way the endpoints fit perfectly
    inacc = (primary_graph_centre_line_x-primary_graph_centre_minimum_x)%primary_span_interval
    target = primary_graph_centre_minimum_x + inacc
    centre_ratio = (target - primary_graph_centre_line_x)/-primary_graph_centre_line_x

    # now start from scratch now with a different ratio
    primary_graph_centre_minimum_x = primary_graph_centre_line_x - centre_ratio*primary_graph_centre_line_x
    primary_graph_centre_maximum_x = primary_graph_centre_line_x + centre_ratio*primary_graph_centre_line_x
    no_elements = (primary_graph_centre_line_x - primary_graph_centre_minimum_x)//primary_span_interval
    primary_graph_centre_minimum_y = primary_graph_centre_line_y - 1000*no_elements
    primary_graph_centre_maximum_y = primary_graph_centre_line_y + 1000*no_elements
    secondary_graph_y_maximum = primary_graph_centre_line_y*2

    # math function for later
    g = primary_graph_centre_minimum_x
    d = primary_graph_centre_minimum_y
    s = primary_graph_default_ascent
    b = (2*d)/g - s
    a = (s-b)/(2*g)

    if b < 0:
        print("Error: Too extreme value!")

    primary_span_grid = []

    def graph_function(y):
        # in central part
        if y >= primary_graph_centre_minimum_y and y <= primary_graph_centre_maximum_y:
            x_value = primary_graph_centre_minimum_x + primary_span_interval * (y-primary_graph_centre_minimum_y)/1000
            return x_value
        # error
        elif y > primary_graph_centre_line_y*2 or y < 0:
            return 2
        # on the one hand; recalls the own function in a mirrored way and executes the last part of the function
        elif y > primary_graph_centre_maximum_y:
            # mirror everything to be on the safe side
            y = secondary_graph_y_maximum - y
            x_value = graph_function(y)
            return secondary_span_length - x_value
        # it's outside the centre: do the differential
        else:
            # unexplainable math function; derived from differential
            x_value = ((b*-1) + math.sqrt(b**2 - 4*a*(-y)))/(2*a)
            if x_value < 1:
                x_value = primary_span_board_width_outside
            return x_value

    # +1 is an unelegant solution that makes sure the highest element is still respected
    for i in range(0,int(secondary_graph_y_maximum)+1, 1000):
        result = graph_function(i)
        if result == 2:
            break
        else:
            primary_span_grid.append(result)
    #return primary_span_grid

    ############################################################
    ############################################################
    ############################################################
    # now comes the long span
    secondary_span_centre = primary_span_length//2

    # helps to determine the graph function
    def longspan_function(val, searched):
        if searched == "y":
            return val**secondary_span_interval_development
        if searched == "x":
            value = val**(1/secondary_span_interval_development)
            return value


    no_secondary_span_elements = secondary_span_length//secondary_span_interval
    # determine the steps
    secondary_graph_y_max = longspan_function(secondary_span_centre, "y")
    secondary_graph_y_min = secondary_graph_y_max * -1
    secondary_graph_y_step = secondary_graph_y_max/((no_secondary_span_elements-1)/2)
    #print(secondary_graph_y_min)

    # go through the function to get the final values
    secondary_graph_y_current = 0
    secondary_graph_y_list_positive = []
    secondary_graph_y_list_final = []
    while secondary_graph_y_current <= secondary_graph_y_max+1:
        # function only works for positive values!
        secondary_graph_y_list_positive.append(longspan_function(secondary_graph_y_current, "x"))
        secondary_graph_y_current += secondary_graph_y_step

    # now create a negative list
    for i in range(len(secondary_graph_y_list_positive)-1, 0, -1):
        secondary_graph_y_list_final.append(secondary_graph_y_list_positive[i]*-1)
    # and now fuse them
    secondary_graph_y_list_final += secondary_graph_y_list_positive
    # and now push it by half the room width
    for val in range(len(secondary_graph_y_list_final)):
        secondary_graph_y_list_final[val] += primary_span_length/2

    #print(secondary_graph_y_list_final)

    secondary_span_grid = [[],[], []]

    for element in range(1, len(secondary_graph_y_list_final)-1, 2):
        secondary_span_grid[0].append(secondary_graph_y_list_final[element])
        secondary_span_grid[1].append(secondary_graph_y_list_final[element])
    for element in range(2, len(secondary_graph_y_list_final)-1, 2):
        secondary_span_grid[0].append(secondary_graph_y_list_final[element])
        secondary_span_grid[2].append(secondary_graph_y_list_final[element])
    for j in range(len(secondary_span_grid)):
        secondary_span_grid[j].insert(0, secondary_span_board_width)
        secondary_span_grid[j].append(primary_span_length - secondary_span_board_width)

    secondary_span_grid[0].sort()
    return primary_span_grid, secondary_span_grid


class Ceilingsystem:
    def __init__(self, grids, skip):
        self.ceiling_grids = grids
        self.timberboards = []
        self.skipping = skip

    # creates the timber boards and equips them with dimensions and z-location
    def data_setup(self):
        z_value = 0
        global_counter = 0
        for layer in range(layer_no):
            self.timberboards.append([])
            # outside layer, primary_span
            if layer == 0 or layer == layer_no - 1:
                boardheight = primary_board_height_outside
                z_value += boardheight
                for i in range(len(self.ceiling_grids[0])):
                    self.timberboards[layer].append(Timberboard(layer, global_counter, i, primary_board_outside_dimensions, z_value))
                    global_counter += 1
            elif layer%2 == 0:
                boardheight = primary_board_height_inside
                z_value += boardheight
                # no dedensification on the inside
                if self.skipping == False:
                    for i in range(len(self.ceiling_grids[0])):
                        self.timberboards[layer].append(Timberboard(layer, global_counter, i, primary_board_inside_dimensions, z_value))
                        global_counter += 1
                # dedensification on the inside
                else:
                    # not sure about the -1
                    max_i = 0
                    for i in range(0, len(self.ceiling_grids[0])-1, 2):
                        self.timberboards[layer].append(Timberboard(layer, global_counter, i, primary_board_inside_dimensions, z_value))
                        global_counter += 1
                        max_i = i
                    self.timberboards[layer].append(Timberboard(layer, global_counter, max_i+1, primary_board_inside_dimensions, z_value))
                    global_counter += 1
            else:
                boardheight = secondary_board_height
                z_value += boardheight
                for i in range(0, len(self.ceiling_grids[1][0])):
                    self.timberboards[layer].append(Timberboard(layer, global_counter, i, secondary_board_dimensions, z_value))
                    global_counter += 1

    # specifies the centre point locations
    def geometry_setup(self):
        for timber_layer in self.timberboards:
            for board in timber_layer:
                primary_direction_coordinate = primary_length / 2
                if board.layer % 2 == 0:
                    if self.skipping and board.layer != 0 and board.layer != layer_no:
                        # unnecessary branch at the moment but we might still need it so.....
                        secondary_direction_coordinate = self.ceiling_grids[0][board.no_in_layer]
                    else:
                        secondary_direction_coordinate = self.ceiling_grids[0][board.no_in_layer]
                else:
                    secondary_direction_coordinate = secondary_length/2
                    primary_direction_coordinate = self.ceiling_grids[1][0][board.no_in_layer]

                # now correct the location in reference to the robot
                primary_direction_coordinate -= robot_origin_primary_direction
                secondary_direction_coordinate -= robot_origin_secondary_direction
                z_direction_coordinate = board.z_drop - robot_origin_z_direction

                board.centre_point[primary_direction] = primary_direction_coordinate
                board.centre_point[secondary_direction] = secondary_direction_coordinate
                board.centre_point[2] = board.z_drop - board.height

                #board.centre_point = board.drop_point - Vector(0, 0, board.dimensions[1]/2)


    # next step: upper/lower neighbours + gluepoints


class Timberboard:
    def __init__(self, board_layer, identification, board_no_in_layer, board_dimensions, z_value_toppoint):
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

        if primary_direction == 0:
            self.primary_vector = (1,0,0)
            self.secondary_vector = (0,1,0)
        else:
            self.primary_vector = (0,1,0)
            self.secondary_vector = (1,0,0)


        self.board_frame = Frame(self.centre_point, self.primary_vector, self.secondary_vector)
        self.box = Box(self.board_frame, self.length, self.width, self.height)



#secondary_span_interval_development: 1 = constant, <1: denser in the centre, >1: denser on the edges
#operable range approximately 0.6/6

layer_no = 5
gap_min = 4.0
primary_length = 450
omnidirectional = True
primary_board_width_outside = 6
primary_board_height_outside = 4
primary_board_width_inside = 6
primary_board_height_inside = 4
primary_board_outside_dimensions = [primary_board_width_outside, primary_board_height_outside, primary_length]
primary_board_inside_dimensions = [primary_board_width_inside, primary_board_height_inside, primary_length]
secondary_board_width = 4
secondary_board_height = 4
secondary_length = 840
secondary_board_dimensions = [secondary_board_width, secondary_board_height, secondary_length]

primary_interval = 12
primary_falloff = 90
primary_dedensification = 3
secondary_interval = 80
secondary_interval_development = .7
skip_centrals = True

primary_direction = 0
secondary_direction = 1
robot_origin_primary_direction = -90
robot_origin_secondary_direction = -10
robot_origin_z_direction = -20
primary_axis = "x"

# calculates the basic centre lines
# basically just a list of numbers, the rest comes later
myGrid = grid_creation(gap_min,primary_length,omnidirectional,primary_board_width_outside,\
primary_board_height_outside,primary_board_width_inside,primary_board_height_inside,\
secondary_length,secondary_board_width, secondary_board_height,primary_interval,\
primary_falloff,primary_dedensification,secondary_interval,secondary_interval_development)

myCeiling = Ceilingsystem(myGrid, skip_centrals)
myCeiling.data_setup()
myCeiling.geometry_setup()

print("done")


