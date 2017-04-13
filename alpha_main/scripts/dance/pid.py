class PID(object):
    def __init__(self, k_p=1.0, k_i=0.0, k_d=0.0):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.e_i = 0.0
        self.e_d = 0.0

    def compute(self,err,dt):
        e_p = err
        e_i = self.e_i
        e_d = err - self.e_d

        self.e_i += err * dt
        self.e_d = err

        return self.k_p * e_p + self.k_i * e_i + self.k_d * e_d
    def reset(self):
        self.e_i = 0.0
        self.e_d = 0.0
