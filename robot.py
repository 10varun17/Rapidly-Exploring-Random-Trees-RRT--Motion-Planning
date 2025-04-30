import numpy as np

class Robot:
    def __init__(self, x0, y0, yaw0, dt=0.001):
        self.q0 = np.array((x0, y0, yaw0), dtype=float)
        self.x_next = x0
        self.y_next = y0
        self.yaw_next = yaw0
        self.q = self.q0
        self.dt = dt

    def get_state(self):
        return self.q
    
    def update_state(self, u):
        v = u[0]
        omega = u[1]

        self.x_next += self.dt * (v * np.cos(self.yaw_next))
        self.y_next += self.dt * (v * np.sin(self.yaw_next))
        self.yaw_next += self.dt * omega

        self.q[0] = self.x_next
        self.q[1] = self.y_next
        self.q[2] = self.yaw_next
    
