class Integrator:
    def __init__(self, dt, f):
        self.dt = dt
        self.f = f

    def step(self, t, x, u):
        raise NotImplementedError

class Euler(Integrator):
    def step(self, t, x, u):
        return x + self.dt * self.f(t, x, u)

class Heun(Integrator):
    def step(self, t, x, u):
        intg = Euler(self.dt, self.f)
        xe = intg.step(t, x, u) # Euler predictor step
        return x + 0.5*self.dt * (self.f(t, x, u) + self.f(t+self.dt, xe, u))

class RK4(Integrator):
    def step(self, t, x, u):
        k_1 = self.f(t, x, u)
        k_2 = self.f(t + self.dt/2, x + self.dt*k_1/2, u)
        k_3 = self.f(t + self.dt/2, x + self.dt*k_2/2, u)
        k_4 = self.f(t + self.dt, x + self.dt*k_3, u)
        return x + self.dt*(k_1 + 2*k_2 + 2*k_3 + k_4)/6
