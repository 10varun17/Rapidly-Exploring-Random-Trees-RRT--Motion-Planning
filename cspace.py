import random

class ConfigSpace:
    def __init__(self, x_bounds=(0,10), y_bounds=(0,10), obstacles=None):
        self.xmin, self.xmax = x_bounds
        self.ymin, self.ymax = y_bounds
        self.obstacles = obstacles if obstacles is not None else []
        if not self.obstacles:
            self.add_obstacles(random.randint(4,6))

    def sample(self):
        """
        Sample a random point in the configuration space.
        """
        x = random.uniform(self.xmin, self.xmax)
        y = random.uniform(self.ymin, self.ymax)
        return (x, y)
    
    def is_in_bounds(self, q):
        """
        Check if the given point is in the configuration space.
        """
        x, y = q
        is_in_xbounds = (self.xmin < x or self.xmin == x) and (x < self.xmax or x == self.xmax)
        is_in_ybounds = (self.ymin < y or self.ymin == y) and (y < self.ymax or y == self.ymax)
        return is_in_xbounds and is_in_ybounds
    
    def add_obstacles(self, num_obstacles):
        """
        Create the given number of obstacles, with random center and radius, but still inside the
        configuration space.
        """
        for _ in range(num_obstacles):
            radius = random.uniform(1., 2.)
            center_x = random.uniform(self.xmin + radius, self.xmax - radius)
            center_y = random.uniform(self.ymin + radius, self.ymax - radius)
            obstacle = ((center_x, center_y), radius)
            self.obstacles.append(obstacle)
    
    def collision_free(self, q1, q2):
        """
        Check if the two given points are not inside the obstacles, including the line segment joining them.
        """
        for obstacle in self.obstacles:
            if self.is_in_obstacle(q1, obstacle) or self.is_in_obstacle(q2, obstacle):
                return False
            
            if self.segment_crosses_obstacle(q1, q2, obstacle):
                return False
            
        return True

    def is_in_obstacle(self, point, obstacle:tuple):
        """
        Check if a given point is inside the given circular obstacle.
        """
        center, radius = obstacle
        dx = point[0] - center[0]
        dy = point[1] - center[1]
        offset = 0.1
        h = dx**2 + dy**2 - (radius + offset)**2
        return h <= 0
    
    def segment_crosses_obstacle(self, A, B, obstacle:tuple):
        """
        Check if the line segment between given points A and B crosses the given circular obstacle.
        """
        center, _ = obstacle
        
        # vector from A to B
        ABx, ABy = B[0] - A[0], B[1] - A[1]

        # vector from A to center (C)
        ACx, ACy = center[0] - A[0], center[1] - A[1]

        # norm of AB
        ab_norm = ABx**2 + ABy**2
        if ab_norm == 0:
            # A and B are same points
            return self.is_in_obstacle(A, obstacle)
        
        # project AC onto AB, parameterized by t, t = (AC.AB)/|AB|^2
        t = (ACx * ABx + ACy * ABy)/ab_norm

        # clamp t to [0, 1] so that projected point D stays on the segment
        t = max(0, min(1, t))

        # find the closest point D = A + t*AB
        dx = A[0] + t*ABx
        dy = A[1] + t*ABy
        D = (dx, dy)

        # check if D is in obstacle
        return self.is_in_obstacle(D, obstacle)