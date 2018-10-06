

import math
def point_in_map(point,map):  # point is [x_cordinate,y_cordinate]

    if point[1] < 0 or point[0] < 0:  # bellow map
        return False

    if point[0] >= len(map[0]) or point[1] >= len(map):  # above map
        return False

    if map[point[1]][point[0]] == 1:  # wall
        return False

    return True


def setup_of_lenght_map(map):
<<<<<<< HEAD
    len_map = []

=======
    global len_map
    
>>>>>>> origin/master
    for line in map:
        new_line = []

        for element in line:
            if element == 1:
                new_line.append(0)
            else:
                new_line.append(10 ** 3)
        len_map.append(new_line)
    return len_map


class PointPath():

    def __init__(self, x, y, prev_path_input):
        self.x_cordinate = x
        self.y_cordinate = y
        self.previous_points = prev_path_input
        self.map = map

    def get_path_length(self):
        total_length = 0.0
        for ind in range(0, len(self.previous_points)):

            max_x = self.x_cordinate
            max_y = self.y_cordinate
            if ind < len(self.previous_points) - 1:
                max_x = self.previous_points[ind + 1][0]
                max_y = self.previous_points[ind + 1][1]
            len_x = float(self.previous_points[ind][0] - max_x)
            len_y = float(self.previous_points[ind][1] - max_y)
            if len_x < 0:
                len_x *= (-1)
            if len_y < 0:
                len_y *= (-1)
            total_length += (len_x ** 2.0 + len_y ** 2.0) ** (0.5)

        return total_length

    def check_for_crash(self,map):

        for ind in range(0, len(self.previous_points)):
            min_x = self.previous_points[ind][0]
            if ind == len(self.previous_points) - 1:
                max_x = self.x_cordinate
                max_y = self.y_cordinate
            else:
                max_x = self.previous_points[ind + 1][0]
                max_y = self.previous_points[ind + 1][1]
            min_y = self.previous_points[ind][1]

            if min_x > max_x:
                temp = max_x
                max_x = min_x
                min_x = temp
            if min_y > max_y:
                temp = max_y
                max_y = min_y
                min_y = temp

            for x_cor in range(min_x, max_x + 1):
                for y_cor in range(min_y, max_y + 1):
                    if not point_in_map([x_cor, y_cor],map):
                        return True  # point not in map or blocked by wall

        return False

    def get_all_new_points(self,map):
        new_point_paths = []
        height = len(map)
        width = len(map[0])
        # UP
        for n in range(1, 18):
            y_offset = 2
            x_offset = n % 3 - 2
            if n > 3:
                y_offset = -2

            if n > 5:
                y_offset = 3
            if n > 8:
                y_offset = -3
            if n > 11:
                y_offset = 4
            if n > 14:
                y_offset = -4

            new_path = []
            for p in self.previous_points:
                new_path.append(p)
            # new_path = self.previous_points
            new_path.append([self.x_cordinate, self.y_cordinate])
            new_point = PointPath(self.x_cordinate + x_offset, self.y_cordinate + y_offset, new_path)
            if not new_point.check_for_crash(map):
                new_point_paths.append(new_point)

        for x_offset in range(-4, 5):
            for y_offset in range(-1, 2):
                if x_offset == 0 and y_offset == 0:
                    continue  # this is the current point, no need to include it.

                new_path = []
                for p in self.previous_points:
                    new_path.append(p)
                # new_path = self.previous_points
                new_path.append([self.x_cordinate, self.y_cordinate])
                new_point = PointPath(self.x_cordinate + x_offset, self.y_cordinate + y_offset, new_path)

                if not new_point.check_for_crash(map):
                    new_point_paths.append(new_point)

        return new_point_paths


def clean_my_point_array(input_array):
    output_array = input_array
    while True:
        didChange = False
        for ind in range(0,len(output_array) - 2):

            x0 = output_array[ind][0]
            x1 = output_array[ind + 1][0]
            x2 = output_array[ind + 2][0]
            y0 = output_array[ind][1]
            y1 = output_array[ind + 1][1]
            y2 = output_array[ind + 2][1]

            if x0 == x1 and x1 == x2:
                didChange = True
            elif y0 == y1 and y1 == y2:
                didChange = True
            elif (x1 - x0) != 0 and (x2 - x1) != 0:
                if ((y1-y0)/(x1 - x0)) == ((y2-y1)/(x2 - x1)):
                    didChange = True

            if didChange:
                del output_array[ind + 1]
                break
        if didChange == False:
            break

    #lengste pne veiene output_array



    return output_array


def add_breaking_points(input_array):


    return_array = []
    didChange = False
    for ind in range(0, len(input_array) - 1):

        x0 = input_array[ind][0]
        x1 = input_array[ind + 1][0]

        y0 = input_array[ind][1]
        y1 = input_array[ind + 1][1]

        road_length = ((x0 - x1)**2 + (y0 - y1)**2)**(0.5)
        return_array.append(input_array[ind])
        if road_length > 3: #here we add a new point
                break_lenght = 1
                if road_length > 4:
                    break_lenght = 1.2

                if x0 == x1:
                    if y1 > y0:
                        return_array.append([x1,y1 - break_lenght])
                    else:
                        return_array.append([x1, y1 + break_lenght])
                elif y0 == y1:
                    if x1 > x0:
                        return_array.append([x1 - break_lenght,y1])
                    else:
                        return_array.append([x1 + break_lenght, y1])
                else:
                    myradians = math.atan2(y1 - y0, x1 - x0)
                    return_array.append([x1 - break_lenght*math.cos(myradians),y1 - break_lenght*math.sin(myradians)])


    return_array.append(input_array[len(input_array) - 1])

    return return_array




def find_shortest_path(map,start_point, end_point):
    setup_of_lenght_map(map)
    len_map = setup_of_lenght_map(map)
    #print("These should be the same")
    #print(len(map))
    #print(len(map[0]))
    #print("as these:")
    #print(len(len_map))
    # print(len(len_map[0]))
    paths_array = [PointPath(start_point[0], start_point[1], [])]

    winner_paths = []

    while len(paths_array) > 0:

        new_paths_array = []

        for path in paths_array:

            new_paths = path.get_all_new_points(map)

            for path in new_paths:
                #print(path.y_cordinate)
                #print(path.x_cordinate)
                if path.get_path_length() < len_map[path.y_cordinate][path.x_cordinate]:

                    for point in path.previous_points:

                        len_map[path.y_cordinate][path.x_cordinate] = path.get_path_length()
                    if end_point[1] == path.y_cordinate and end_point[0] == path.x_cordinate:

                        winner_paths.append(path)
                    else:

                        new_paths_array.append(path)

        paths_array = new_paths_array

    newPoint = []
    newPoint.append(winner_paths[len(winner_paths) - 1].x_cordinate)
    newPoint.append(winner_paths[len(winner_paths) - 1].y_cordinate)
    winner_paths[len(winner_paths) - 1].previous_points.append(newPoint)
    clean_array = clean_my_point_array(winner_paths[len(winner_paths) - 1].previous_points)
    return add_breaking_points(clean_array)
    #return winner_paths[len(winner_paths) - 1].previous_points


"""
x = add_breaking_points([[0,0],[7,0],[3,3],[19,-13],[0,0],[-3,-3],[3,-7],[15,19],[100,-100]])

print(x)
"""