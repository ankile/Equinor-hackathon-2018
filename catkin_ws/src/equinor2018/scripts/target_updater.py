
class TargetUpdater:
    def __init__(self, path):
        self.path = path

    def should_move_to_next_target(self, x0, y0):
        if not self.path:
            return False

        (x, y) = self.path[0]
        distance = ((x - x0)**2 + (y - y0)**2) ** 0.5

        return distance < 0.3

    def next_target(self):
        next = self.path[0]
        self.path = self.path[1:]

        return next