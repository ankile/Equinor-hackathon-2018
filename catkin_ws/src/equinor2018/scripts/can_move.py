


def magnitude(source, destination):
    (x0, y0) = source
    (x1, y1) = destination
    return ((x1 - x0)**2 + (y1 - y0)**2) ** 0.5

def norm(t):
    m = t[0]**2 + t[1]**2
    return (t[0] / m, t[1] / m)

def approximate_time_to(source, destination):
    distance = magnitude(source, destination)
    # We reach constant speed after approx. 0.5s
    # at which point we've travelled about 4.4 meters.
    distance_at_constant_speed = distance - 4.426887416184

    if distance_at_constant_speed > 0:
        # We travel at about 26.50781333531 m/s at full speed
        max_speed = 26.50781333531
        return 0.5 + distance_at_constant_speed / max_speed

    # Found by solving s(t) = at^2 + bt + c for t. Where s(t) is found through line-fitting.
    return 5.221492994603 * (10**(-8)) * (1.354223364692 * (10**13) * distance + 963746629199) ** 0.5 + 0.0924769712965

def is_collision(grid, point):
    (x, y) = point
    return grid[int(y)][int(x)] in [0, '\x00']


def line_is_non_colliding(grid, start, velocity_sum, lookahead, pieces = 1000):
    scale = lambda x, t: (x * t[0], x * t[1])

    for i in range(1.0, pieces + 1):
        (x0, y0) = start
        (x, y) = velocity_sum
        p = (x0 + 1.0 * x / pieces, y0 + 1.0 * y / pieces)

        if is_collision(grid, p):
            return False

    return True



# We'll assume that the initial velocity vector is constant throughout the movement, even though that isn't the case.
def is_safe_to_move(grid, source, destination, velocity, lookahead = 5, drone_size = 0.9):
    drone_width = drone_size * 0.5
    (x0, y0) = source
    (x1, y1) = destination
    (x, y) = (x1 - x0 + velocity[0], y1 - y0 + velocity[1])
    (dx, dy) = norm((- y/x, 1) if abs(x) > abs(y) else (1, - x / y))

    first_is_non_colliding = line_is_non_colliding(grid, (x0 + drone_width * dx, y0 + drone_width * dy), (dx,dy), lookahead)
    second_is_non_colliding = line_is_non_colliding(grid, (x0 - drone_width * dx, y0 + drone_width * dy), (dx,dy), lookahead)

    return first_is_non_colliding and second_is_non_colliding


