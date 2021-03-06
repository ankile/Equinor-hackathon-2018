def get_mid_breakpoints(input_array):
    return_array = []
    for ind in range(0, len(input_array) - 1):
        return_array.append(input_array[ind])
        x0 = input_array[ind][0]
        x1 = input_array[ind + 1][0]

        y0 = input_array[ind][1]
        y1 = input_array[ind + 1][1]

        if x0 == x1:
            if y1 > y0:
                return_array.append([x1, y1 - 0.5])
            else:
                return_array.append([x1, y1 + 0.5])
        elif y0 == y1:
            if x1 > x0:
                return_array.append([x1 - 0.5, y1])
            else:
                return_array.append([x1 + 0.5, y1])
    return return_array



def insert_break_points(input_array):
    return_array = []
    didChange = False
    for ind in range(0, len(input_array) - 1):

        x0 = input_array[ind][0]
        x1 = input_array[ind + 1][0]

        y0 = input_array[ind][1]
        y1 = input_array[ind + 1][1]

        road_length = ((x0 - x1)**2 + (y0 - y1)**2)**(0.5)
        return_array.append(input_array[ind])
        if road_length >= 2: #here we add a new point
                break_lenght = 0.8
                if road_length >= 3:
                    break_lenght = 2.2
                if road_length >= 4:
                    break_lenght = 2.8

                if road_length >= 5:
                    break_lenght = 5.2

                if road_length >= 8:
                    break_lenght = 6.2

                if road_length >= 12:
                    break_lenght = 9

                if x0 == x1:
                    if y1 > y0:
                        return_array.append([x1,y1 - break_lenght])
                        if break_lenght > 2:
                            return_array.append([x1, y1 - break_lenght/3])
                        if break_lenght > 7:
                            return_array.append([x1, y1 - break_lenght / 4])
                    else:
                        return_array.append([x1, y1 + break_lenght])
                        if break_lenght > 2:
                            return_array.append([x1, y1 + break_lenght/3])
                        if break_lenght > 7:
                            return_array.append([x1, y1 + break_lenght/4])
                elif y0 == y1:
                    if x1 > x0:
                        return_array.append([x1 - break_lenght,y1])
                        if break_lenght > 2:
                            return_array.append([x1 - break_lenght/3, y1])
                        if break_lenght > 7:
                            return_array.append([x1 - break_lenght/4, y1])
                    else:
                        return_array.append([x1 + break_lenght, y1])
                        if break_lenght > 2:
                            return_array.append([x1 + break_lenght / 3, y1])
                        if break_lenght > 7:
                            return_array.append([x1 + break_lenght / 4, y1])


    return_array.append(input_array[len(input_array) - 1])

    return return_array
