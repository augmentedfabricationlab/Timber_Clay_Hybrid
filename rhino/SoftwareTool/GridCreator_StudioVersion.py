import math
import copy

from compas.geometry import Point


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

    secondary_span_grid = [[],[]]

    for element in range(1, len(secondary_graph_y_list_final)-1, 2):
        secondary_span_grid[0].append(secondary_graph_y_list_final[element])
    for element in range(2, len(secondary_graph_y_list_final)-1, 2):
        secondary_span_grid[1].append(secondary_graph_y_list_final[element])
    for j in range(len(secondary_span_grid)):
        secondary_span_grid[j].insert(0, secondary_span_board_width)
        secondary_span_grid[j].append(primary_span_length - secondary_span_board_width)

    return primary_span_grid, secondary_span_grid



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
secondary_length = 840
secondary_board_width = 4
secondary_board_height = 4
primary_interval = 12
primary_falloff = 90
primary_dedensification = 3
secondary_interval = 60
secondary_interval_development = .4
skip_centrals = True


# calculates the basic centre lines
# basically just a list of numbers, the rest comes later
myGrid = grid_creation(gap_min,primary_length,omnidirectional,primary_board_width_outside,\
primary_board_height_outside,primary_board_width_inside,primary_board_height_inside,\
secondary_length,secondary_board_width, secondary_board_height,primary_interval,\
primary_falloff,primary_dedensification,secondary_interval,secondary_interval_development)



myCeiling = Ceilingsystem(myGrid)
myCeiling.setup(skip_centrals)

class Ceilingsystem:
    def __init__(self, grids):
        self.ceiling_grids = grids
        self.timberboards = []

    def setup(skip=False):
        for layer in range(layer_no):
            # outside layer, primary_span
            if layer == 0 or layer == layer_no - 1:
                for i in range(self.ceiling_grids[0]):
                    self.timberboards.append(Timberboard(layer, i))
            
                    
                

class Timberboard:
    def __init__(self, board_layer, board_no_in_layer, skip = False):        
        self.layer = board_layer
        self.no_in_layer = board_no_in_layer

        
    
    def board_dimensions():
        if self.layer == 0 or self.layer == (layer_no-1):
        # inside layer, primary span
        elif layer % 2 == 0:
        # inside layer, secondary span
        else:

        self.width = 0
        self.length = 0
        self.height = 
    

"""
    # now get everything into layers
    droppoints_layers = []
    for i in range(no_layers):
        droppoints_layers.append([])
        pt_x = primary_span_length/2
        pt_z = i * ((primary_span_board_height_outside+secondary_span_board_height)/2) + primary_span_board_height_outside
        block_length = primary_span_length
        if i%2 == 0:
            # first/last layer --> be dense        
            if i == 0 or i == no_layers-1:
                for j in range(0, len(primary_span_nts)):
                    pt_y = (primary_span_nts[j])
                    droppoints_layers[i].append([[pt_x, pt_y, pt_z], block_length])
            else:
                # all the other layers
                # first point
                pt_y = shortspanpoints_odd[0]
                droppoints_layers[i].append([[pt_x, pt_y, pt_z], block_length])
                if (i/2)%2 == 0:
                    start = 1                
                else:
                    start = 2
                for g in range(start, len(shortspanpoints_odd)-1, 2):
                    pt_y = shortspanpoints_odd[g]
                    droppoints_layers[i].append([[pt_x, pt_y, pt_z], block_length])                
                # last point
                pt_y = shortspanpoints_odd[-1]
                droppoints_layers[i].append([[pt_x, pt_y, pt_z], block_length])        
        else:
            continue

    #print(droppoints_layers[0])



    #print(longspan_centrelines_even)
    #print(longspan_centrelines_odd)

    ### get a pattern out of the centre lines
    s = short_block_length
    l = long_block_length
    pattern1 = [s, l, s, s, s, s]
    pattern2 = [l, s, l, s, s]

    secondary_span_length_extended = secondary_span_length*1.1


    # create the final pattern one

    def longspan_pattern_definer(pattern, adapted_length):
        output_pattern = []
        centre_points = []
        total_length = 0
        no_longspan_blocks = -1
        # set up the pattern
        while total_length < adapted_length:
            no_longspan_blocks += 1
            # if we have shot over the pattern
            if no_longspan_blocks > (len(pattern)-1):
                # if u can end it with a big one then do that:
                if total_length > adapted_length - l:
                    total_length += l                
                    output_pattern.append(long_block_length)
                    centre_points.append(total_length - long_block_length/2)
                    break
                else:
                    total_length += s                
                    output_pattern.append(250)
                    centre_points.append(total_length - short_block_length/2)
            else:
                total_length += pattern[no_longspan_blocks]
                if pattern[no_longspan_blocks] == long_block_length:
                    output_pattern.append(long_block_length)
                    centre_points.append(total_length - long_block_length/2)
                else:
                    output_pattern.append(short_block_length)
                    centre_points.append(total_length - short_block_length/2)
        # now scale the centre points down
        factor = secondary_span_length/total_length
        for pt in range(1, len(centre_points)-1):
            centre_points[pt] *= factor
            centre_points[-1] = secondary_span_length - output_pattern[-1]/2
        return output_pattern, centre_points


    # now same thing on the sides
    def longspan_pattern_definer_sides(primary, secondary):
        total_length = 0
        iteration = 0
        pattern = []
        centre_points_side = []
        while total_length < secondary_span_length:
            # situation at the beginning
            if iteration == 0:
                centre_point = primary/2
                total_length += primary
                pattern.append(primary)
                centre_points_side.append(centre_point)
                iteration += 1
            else:
                # check whether we are already approaching the end
                if secondary_span_length - total_length < short_block_length:
                    break
                elif secondary_span_length - total_length < long_block_length:
                    piece = short_block_length
                elif secondary_span_length - total_length < long_block_length + short_block_length:
                    piece = long_block_length
                # assuming we have not approached the end yet, then go in here
                elif iteration%2 == 1:
                    piece = short_block_length
                else:
                    piece = long_block_length
                # now just append everything
                centre_point += piece/2
                centre_point += pattern[iteration-1]/2
                total_length += piece
                pattern.append(piece)
                centre_points_side.append(centre_point)
                iteration += 1
        # after the loop extend the points so everything fits well

        initial_length = pattern[0]/2
        last_length = pattern[-1]/2
        factor = (secondary_span_length-initial_length-last_length)/(total_length-initial_length-last_length)
        
        # primary_graph_centre_line_x = secondary_span_length/2
        for c in range(len(centre_points_side)):
            position = centre_points_side[c]
            centre_points_side[c] = initial_length + (position-initial_length)*factor
    
        
        # analysis of the gaps
        gaps = []
        for c in range(len(centre_points_side)):
            upper_end = centre_points_side[c] + pattern[c]/2
            lower_end = centre_points_side[c] - pattern[c]/2
            gaps.append([lower_end, upper_end])
        print(gaps)

        #print(centre_points_side)
        return pattern, centre_points_side



    # call the pattern creator to set up the actual patterns
    longspanpatterns = []
    longspanpatterns.append(longspan_pattern_definer(pattern1, secondary_span_length_extended))
    longspanpatterns.append(longspan_pattern_definer(pattern2, secondary_span_length_extended))

    #print(longspan_centrelines)
    sidepatterns = []
    sidepatterns.append(longspan_pattern_definer_sides(short_block_length, long_block_length))
    sidepatterns.append(longspan_pattern_definer_sides(long_block_length, short_block_length))


    # now set up odd and even layers
    layer = 0
    # go through the layers
    for layer in range(1, no_layers, 2):
        # make sure that stuff alterates
        if layer%4 == 1:
            starting_id = 0
        else:
            starting_id = 1
        # um die patterns regelmaessig zu switchen; vorsicht fehlerquelle!
        pattern_id = copy.deepcopy(starting_id)
        a = len(longspan_centrelines[starting_id])
        
        for i in range(len(longspan_centrelines[starting_id])):

            # if we are on the sides
            if i == 0 or i == len(longspan_centrelines[starting_id])-1:
                # go through first pattern
                if i == 0:
                    # centre_id is just there to hit the right centre line
                    # pat_id makes sure that the pattern switches between the layers
                    pt_x = secondary_span_board_width/2
                else:
                    pt_x = primary_span_length - secondary_span_board_width/2

                if (layer+1)%4 == 0:
                    pat_ids = [0,1]
                else:
                    pat_ids = [1,0]
                # gotta do it twice
                # first with y minus
                for center in range(len(sidepatterns[pat_ids[0]][1])):
                    pt_y = sidepatterns[pat_ids[0]][1][center]
                    pt_z = layer * ((primary_span_board_height_outside+secondary_span_board_height)/2) + secondary_span_board_height
                    block = sidepatterns[pat_ids[0]][0][center]
                    pt = [pt_x, pt_y, pt_z]
                    droppoints_layers[layer].append([pt, block])
                # second with y plus
                #pt_x = sidepatterns[pat_ids[1 = certain pattern of the layer]][1 = centerpoints][center = specific point]
                #for center in range(len(sidepatterns[pat_ids[1]][1])):
                    #pt_y = sidepatterns[pat_ids[1]][1][center]
                    #pt_x = longspan_centrelines[pat_ids[1]][centre_id] + (secondary_span_board_width/2)*1.1
                    #pt_z = layer * ((primary_span_board_height_outside+secondary_span_board_height)/2) + secondary_span_board_height
                    #block = sidepatterns[pat_ids[1]][0][center]
                    #pt = [pt_x, pt_y, pt_z]
                    #droppoints_layers[layer].append([pt, block])

            # if it's everything but the sides
            else:            
                pt_x_centre = longspan_centrelines[starting_id][i]
                pattern_id %= 2
                for j in range(len(longspanpatterns[pattern_id][1])):
                    if j%2 == 0:
                        pt_x = pt_x_centre + secondary_span_board_width/1.7
                    else:
                        pt_x = pt_x_centre - secondary_span_board_width/1.7
                    pt_y = longspanpatterns[pattern_id][1][j]        
                    pt_z = layer * ((primary_span_board_height_outside+secondary_span_board_height)/2) + secondary_span_board_height
                    pt = [pt_x, pt_y, pt_z]
                    block = longspanpatterns[pattern_id][0][j]
                    droppoints_layers[layer].append([pt, block])

    #print(droppoints_layers)


    ############################################################################################################
    ############################################################################################################
    # Gluegun part
    ############################################################################################################
    ############################################################################################################

    # starts at one because first layer does not get glued
    glue_points = [[]]
    for layer in range(1, no_layers):
        # make a new layer for the gluepoints; first layer remains empty
        glue_points.append([])
        # make a slot for every element
        for el in range(len(droppoints_layers[layer])):
            glue_points[layer].append([])
        
        # even layer
        #################################################################
        #there might be an error in the even part; at least some layers seem to be doubled; that yet has to be checked, though
        #################################################################
        if layer%2 == 0:
            # go through the shortspan elements within the layer        
            for droppoint in range(len(droppoints_layers[layer])):
                pt = droppoints_layers[layer][droppoint][0]
                pt_x = pt[0]
                pt_y = pt[1]
                # define the search radius on the layer underneath
                pt_secondary_graph_y_min = pt_y - long_block_length/2
                pt_secondary_graph_y_max = pt_y + long_block_length/2            
                for el in range(len(droppoints_layers[layer-1])):                
                    to_glue = False
                    entry = droppoints_layers[layer-1][el]
                    # rough check
                    centre_pt_y = entry[0][1]
                    if centre_pt_y > pt_secondary_graph_y_min and centre_pt_y < pt_secondary_graph_y_max:
                        # detailed check
                        element_length = entry[1]/2
                        centre_pt_x = entry[0][0]
                        centre_pt_z = entry[0][2]
                        if centre_pt_y < pt_y:
                            if centre_pt_y + element_length > pt_y:
                                to_glue = True
                        else:
                            if centre_pt_y - element_length < pt_y:
                                to_glue = True
                    if to_glue:                       
                        glue_pt = [centre_pt_x, pt_y, centre_pt_z]
                        glue_points[layer][droppoint].append(glue_pt)
        # odd layer
        else:
            for droppoint in range(len(droppoints_layers[layer])):
                #glue_points[layer].append([])                       
                pt = droppoints_layers[layer][droppoint][0]            
                pt_x = pt[0]
                pt_y = pt[1]
                element_length = droppoints_layers[layer][droppoint][1]
                pt_secondary_graph_y_max = pt[1] + element_length/2
                pt_secondary_graph_y_min = pt[1] - element_length/2
                # now check for shortspans in the layer underneath
                for el in range(len(droppoints_layers[layer-1])):                 
                    entry = droppoints_layers[layer-1][el]
                    centre_pt_y = entry[0][1]
                    centre_pt_z = entry[0][2]
                    if centre_pt_y > pt_secondary_graph_y_min and centre_pt_y < pt_secondary_graph_y_max:
                        glue_pt = [pt_x, centre_pt_y, centre_pt_z]
                        glue_points[layer][droppoint].append(glue_pt)

    

    return droppoints_layers, glue_points

"""