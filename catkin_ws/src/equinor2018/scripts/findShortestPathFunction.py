len_map = []


def point_in_map(point,map):  # point is [x_cordinate,y_cordinate]

    if point[1] < 0 or point[0] < 0:  # bellow map
        return False

    if point[0] >= len(map[0]) or point[1] >= len(map):  # above map
        return False

    if map[point[1]][point[0]] == 1:  # wall
        return False

    return True


def setup_of_lenght_map(map):
    global len_map
    len_map
    for line in map:
        new_line = []

        for element in line:
            if element == 1:
                new_line.append(0)
            else:
                new_line.append(10 ** 3)
        len_map.append(new_line)


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


def find_shortest_path(map,start_point, end_point):
    setup_of_lenght_map(map)
    paths_array = [PointPath(start_point[0], start_point[1], [])]

    winner_paths = []

    while len(paths_array) > 0:

        new_paths_array = []

        for path in paths_array:

            new_paths = path.get_all_new_points(map)

            for path in new_paths:

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
    return winner_paths[len(winner_paths) - 1].previous_points


the_answer = find_shortest_path([[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]],[0, 0], [5, 4])

for point in the_answer:
    print(point)
    print("->")
