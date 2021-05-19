from ortools.linear_solver import pywraplp
import numpy as np

P = 3
N = 4
K = 2
d = [[0, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 2, 2, 9],
     [3, 0, 5, 2, 4, 2, 3, 3, 4, 3, 5, 6, 7, 2, 9],
     [3, 4, 0, 2, 1, 4, 3, 3, 4, 3, 5, 6, 1, 2, 9],
     [3, 4, 5, 0, 4, 2, 2, 3, 4, 3, 5, 6, 7, 3, 8],
     [3, 7, 1, 2, 0, 4, 3, 3, 4, 2, 5, 5, 7, 2, 9],
     [3, 4, 5, 2, 4, 0, 3, 6, 4, 3, 5, 4, 7, 2, 4],
     [3, 4, 1, 2, 4, 4, 0, 3, 4, 3, 5, 6, 7, 2, 9],
     [3, 3, 5, 3, 7, 1, 3, 0, 4, 8, 5, 5, 4, 2, 3],
     [3, 4, 2, 2, 4, 4, 5, 3, 0, 3, 5, 6, 7, 2, 9],
     [1, 4, 5, 4, 4, 1, 3, 1, 4, 0, 5, 9, 8, 2, 3],
     [3, 4, 2, 2, 5, 4, 3, 3, 4, 3, 0, 6, 7, 2, 9],
     [2, 5, 5, 1, 4, 5, 7, 1, 4, 3, 5, 0, 7, 2, 3],
     [3, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 0, 2, 9],
     [2, 4, 5, 1, 6, 8, 3, 8, 4, 3, 5, 6, 6, 0, 9],
     [3, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 7, 2, 0]]

# with open('project/test.txt', 'r') as file:
#     P, N, K = [int(x) for x in file.readline().split()]
#     r = [int(x) for x in file.readline().split()]
#     Q = [int(x) for x in file.readline().split()]
#     d = [[int(i) for i in file.readline().split()] for j in range(2*P+2*N+1)]

def expand_start(d):
    n = 2*(N+P)
    n_expend = 3*(N+P)+K
    newd = np.zeros((n_expend+1, n_expend+1), dtype=int)

    for i in range(1, n+1):
        for j in range(1, n+1):
            newd[i][j] = d[i][j]

    for i in range(n+1, n_expend+1):
        for j in range(1, n+1):
            newd[i][j] = d[0][j]

    for i in range(1, n+1):
        for j in range(n+1, n_expend+1):
            newd[i][j] = d[i][0]

    return newd


d = expand_start(d)
# print(d)
r = [0, 0, 0, 0, 4, 3, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
cap = [0, 10, 10]

solver = pywraplp.Solver.CreateSolver('CBC')
INF = 10000

# pre-processing
B = {i for i in range(1, 3*(P+N)+K+1)}
# print(B)
F1 = set()
F2 = set()
F3 = set()
B2 = set()
for i in B:
    for k in range(1, K+1):
        F1.add((i, k+2*(N+P)))
        F2.add((k+3*(N+P), i))
    F3.add((i, i))
for i in B:
    for j in B:
        B2.add((i, j))
A = B2 - F1 - F2 - F3
# print(A)
Ap = [[] for i in range(len(A))]  # A+(i)
Am = [[] for i in range(len(A))]  # A-(i)
for (i, j) in A:
    Ap[i].append(j)
    Am[j].append(i)

x = {}
for k in range(1, K+1):
    for (i, j) in A:
        x[k, i, j] = solver.IntVar(
            0, 1, 'x(' + str(k) + ',' + str(i) + ',' + str(j) + ')')

p = {}
for k in range(1, K+1):
    for i in B:
        p[k, i] = solver.IntVar(0, 1, 'p(' + str(k) + ',' + str(i) + ')')

y = {}
for k in range(1, K+1):
    for i in B:
        y[k, i] = solver.IntVar(0, INF, 'y(' + str(k) + ',' + str(i) + ')')

z = {}
for i in B:
    z[i] = solver.IntVar(0, INF, 'z(' + str(i) + ')')

# Constraints
for i in range(1, 2*(N+P)+1):
    c = solver.Constraint(1, 1)
    for k in range(1, K+1):
        for j in Ap[i]:
            c.SetCoefficient(x[k, i, j], 1)

    c = solver.Constraint(1, 1)
    for k in range(1, K+1):
        for j in Am[i]:
            c.SetCoefficient(x[k, j, i], 1)


for i in range(1, 2*(N+P)+1):
    for k in range(1, K+1):
        c = solver.Constraint(0, 0)
        for j in Ap[i]:
            c.SetCoefficient(x[k, i, j], 1)
        for j in Am[i]:
            c.SetCoefficient(x[k, j, i], -1)


# ----------------------------------------------------------------
for k in range(1, K+1):
    c = solver.Constraint(1, 1)
    for j in range(1, 2*(N+P)+1):
        c.SetCoefficient(x[k, k+2*(N+P), j], 1)

    c = solver.Constraint(1, 1)
    for j in range(1, 2*(N+P)+1):
        c.SetCoefficient(x[k, j, k+3*(N+P)], 1)

#
for k in range(1, K+1):
    for (i, j) in A:
        c = solver.Constraint(-INF, INF)
        c.SetCoefficient(x[k, i, j], -INF)
        c.SetCoefficient(z[i], 1)
        c.SetCoefficient(z[j], -1)


for k in range(1, K+1):
    for (i, j) in A:
        c = solver.Constraint(-INF, INF)
        c.SetCoefficient(x[k, i, j], -INF)
        c.SetCoefficient(z[i], -1)
        c.SetCoefficient(z[j], 1)


# -------------
for k in range(1, K+1):
    for (i, j) in A:
        if j <= P:
            c = solver.Constraint(-INF+1, INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(p[k, j], 1)
            c.SetCoefficient(p[k, i], -1)

            c = solver.Constraint(-INF-1, INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(p[k, j], -1)
            c.SetCoefficient(p[k, i], 1)
        elif j <= P+N:
            c = solver.Constraint(-INF+r[j], INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(y[k, j], 1)
            c.SetCoefficient(y[k, i], -1)

            c = solver.Constraint(-INF-r[j], INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(y[k, j], -1)
            c.SetCoefficient(y[k, i], 1)
        elif j <= P+N+P:
            c = solver.Constraint(-INF-1, INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(p[k, j], 1)
            c.SetCoefficient(p[k, i], -1)

            c = solver.Constraint(-INF+1, INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(p[k, j], -1)
            c.SetCoefficient(p[k, i], 1)
        elif j <= (N+P)*2:
            c = solver.Constraint(-INF-r[j], INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(y[k, j], 1)
            c.SetCoefficient(y[k, i], -1)

            c = solver.Constraint(-INF+r[j], INF)
            c.SetCoefficient(x[k, i, j], -INF)
            c.SetCoefficient(y[k, j], -1)
            c.SetCoefficient(y[k, i], 1)
# ----------------------------------------------------------------
for k in range(1, K+1):
    c = solver.Constraint(0, cap[k])
    c.SetCoefficient(y[k, k+3*(N+P)], 1)

for k in range(1, K+1):
    c = solver.Constraint(0, 0)
    c.SetCoefficient(y[k, k+2*(N+P)], 1)

for k in range(1, K+1):
    c = solver.Constraint(0, 1)
    c.SetCoefficient(p[k, k+3*(N+P)], 1)

for k in range(1, K+1):
    c = solver.Constraint(0, 0)
    c.SetCoefficient(p[k, k+2*(N+P)], 1)

# -------------
for k in range(1, K+1):
    c = solver.Constraint(k, k)
    c.SetCoefficient(z[k+2*(N+P)], 1)

    c = solver.Constraint(k, k)
    c.SetCoefficient(z[k+3*(N+P)], 1)

# Objective
obj = solver.Objective()
for k in range(1, K+1):
    for (i, j) in A:
        obj.SetCoefficient(x[k, i, j], int(d[i,j]))

obj.SetMinimization()

result_status = solver.Solve()
assert result_status == pywraplp.Solver.OPTIMAL

print('optimal objective value: %.2f' % solver.Objective().Value())

# print route
for k in range(1, K+1):
    rs = set()
    for (i, j) in A:
        if x[k, i, j].solution_value() > 0:
            rs.add((y[k, i].solution_value(), i))
            rs.add((y[k, j].solution_value(), j))
    route = []
    for i in rs:
        route.append(i)
    route.sort()
    trace = []
    for i in route:
        trace.append(i[1])
    print("Vehicle", k, ":", trace)
