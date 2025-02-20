import numpy as np

class Constraint:
    def __init__(self, compliance=0.0, lambda_=0.0):
        self.compliance = compliance
        self.lambda_ = lambda_
    def solve(self, *args, **kwargs):
        pass
    def solve_velocity(self, *args, **kwargs):
        pass
    def reset(self):
        self.lambda_ = 0.0
        
class DistanceConstraint(Constraint):
    def __init__(self, body1, id1, body2, id2, rest_length, compliance=0.0, lambda_=0.0):
        self.body1 = body1
        self.w1 = self.body1.inv_mass
        self.id1 = id1
        
        self.body2 = body2
        self.w2 = self.body2.inv_mass
        self.id2 = id2
        
        self.rest_length = rest_length
        self.compliance = compliance
        self.lambda_ = lambda_
            
    def solve(self, h):
        # --------------------------------------------------
        # TODO (3) : Distance Constraints
        # --------------------------------------------------
        x1, x2 = self.body1.curr_pos[self.id1], self.body2.curr_pos[self.id2]
        normal = x1 - x2
        length = np.linalg.norm(normal)
        if length < 1e-6:
            return
        
        C = length - self.rest_length
        dC1 = normal / length
        dC2 = -dC1
        if abs(C) < 1e-6:
            return
        
        # Make Constraint Soft!
        alpha = self.compliance / h / h
        dlambda = - (C + alpha * self.lambda_) / (self.w1 + self.w2 + alpha)
        self.lambda_ += dlambda

        dx1 = self.w1 * dlambda * dC1
        dx2 = self.w2 * dlambda * dC2
        
        self.body1.curr_pos[self.id1] += dx1
        self.body2.curr_pos[self.id2] += dx2        

    

class GroundCollisionConstraint(Constraint):
    def __init__(self, body, bottom_vertices, ground_level=0.0, compliance=0.00001):
        """
        Ensure only the bottom face of the lower leg touches the ground.

        :param body: The lower leg object
        :param bottom_vertices: List of vertex indices corresponding to the bottom face
        :param ground_level: The height of the ground
        :param compliance: Compliance parameter for soft constraints
        """
        self.body = body
        self.bottom_vertices = bottom_vertices if isinstance(bottom_vertices, list) else [bottom_vertices]
        self.ground_level = ground_level
        self.w = self.body.inv_mass
        self.n = np.array([0, 1, 0], dtype=np.float32)  # Ground normal vector
        self.compliance = compliance

    def solve(self, h):
        for i in self.bottom_vertices:
            if i >= len(self.body.curr_pos):  # ✅ Ensure index is within range
                continue  

            x = self.body.curr_pos[i].copy()  # ✅ Avoid modifying directly

            # ✅ If already above ground, do nothing (let gravity act naturally)
            if x[1] >= self.ground_level:
                continue

            # ✅ Apply a soft ground constraint instead of forcing it
            C = x[1] - self.ground_level  
            dC = self.n
            alpha = self.compliance / h / h
            dlambda = -C / (self.w + alpha)  
            dx = dlambda * dC

            # ✅ Adjust only the bottom face positions safely
            self.body.curr_pos[i] += dx  

    
def solve_velocity(self):
    # ✅ Apply friction and restitution to all bottom vertices
    for i in self.bottom_vertices:
        v = self.body.vel[i]  # ✅ Now we correctly iterate over vertices
        k_f = self.body.friction
        k_r = self.body.restitution

        v_n = np.dot(v, self.n) * self.n
        v_t = v - v_n

        self.body.vel[i] = - v_n * k_r + v_t * k_f  

        
    
class AttachmentConstraint(Constraint):
    def __init__(self, body, id, anchor, compliance=0.0, lambda_=0.0):
        self.body = body
        self.id = id
        self.w = self.body.inv_mass
        
        self.init_anchor = np.array(anchor, dtype=np.float32)
        self.anchor = self.init_anchor.copy()
        
        self.compliance = compliance
        self.lambda_ = lambda_       
        
        
    def solve(self, h):
        if self.anchor is None:
            return
        # --------------------------------------------------
        # (2) Attach Constraints
        # - C(x) = ||x - anchor||
        # - dC(x) = (x - anchor) / ||x - anchor||
        # --------------------------------------------------        
        x = self.body.curr_pos[self.id]
        d = x - self.anchor
        length = np.linalg.norm(d)
        
        C = length
        if length < 1e-6:
            return
        dC = d / length
        
        # Make Constraint Soft!
        alpha = self.compliance / h / h
        dlambda = -C / (self.w + alpha)
        dx = dlambda * dC
        
        self.body.curr_pos[self.id] += self.w * dx
        
    
    def reset(self):
        super().reset()
        self.anchor = self.init_anchor.copy()

class FixedHeightConstraint(Constraint):
    """Keeps the object at a fixed height, allowing only horizontal movement."""
    def __init__(self, body, fixed_height, compliance=0.00000001):
        self.body = body
        self.fixed_height = fixed_height
        self.compliance = compliance

    def solve(self, h):
        avg_height = np.mean(self.body.curr_pos[:, 1])  # Compute the average height of the object
        height_diff = avg_height - self.fixed_height

        # Apply a correction to keep the torso at the target height
        self.body.curr_pos[:, 1] -= height_diff * (1 - self.compliance)

class MinDistanceConstraint(Constraint):
    def __init__(self, body1, id1, body2, id2, min_length, compliance=0.0, lambda_=0.0):
        self.body1 = body1
        self.w1 = self.body1.inv_mass
        self.id1 = id1
        
        self.body2 = body2
        self.w2 = self.body2.inv_mass
        self.id2 = id2
        
        self.min_length = min_length
        self.compliance = compliance
        self.lambda_ = lambda_
            
    def solve(self, h):
        # --------------------------------------------------
        # TODO (3) : Distance Constraints
        # --------------------------------------------------
        x1, x2 = self.body1.curr_pos[self.id1], self.body2.curr_pos[self.id2]
        normal = x1 - x2
        length = np.linalg.norm(normal)
        if length < 1e-6:
            return
        if length > self.min_length:
            return

        C = length - self.min_length
        dC1 = normal / length
        dC2 = -dC1
        if abs(C) < 1e-6:
            return
        
        # Make Constraint Soft!
        alpha = self.compliance / h / h
        dlambda = - (C + alpha * self.lambda_) / (self.w1 + self.w2 + alpha)
        self.lambda_ += dlambda

        dx1 = self.w1 * dlambda * dC1
        dx2 = self.w2 * dlambda * dC2
        
        self.body1.curr_pos[self.id1] += dx1
        self.body2.curr_pos[self.id2] += dx2        

