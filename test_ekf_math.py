import sympy as sp
dt = sp.Symbol('dt')
P00, P01, P02 = sp.Symbol('P00'), sp.Symbol('P01'), sp.Symbol('P02')
P10, P11, P12 = sp.Symbol('P10'), sp.Symbol('P11'), sp.Symbol('P12')
P20, P21, P22 = sp.Symbol('P20'), sp.Symbol('P21'), sp.Symbol('P22')

P = sp.Matrix([[P00, P01, P02],
               [P10, P11, P12],
               [P20, P21, P22]])

F = sp.Matrix([[1, dt, -0.5*dt**2],
               [0,  1, -dt],
               [0,  0,  1]])

Q = sp.Matrix([[0.001, 0, 0],
               [0, 0.01, 0],
               [0, 0, 0.0001]])

P_next = F * P * F.T + Q
for i in range(3):
    for j in range(3):
        print(f"P[{i}][{j}] = {sp.simplify(P_next[i,j])};")
