from ortools.linear_solver import pywraplp
import numpy as np

with open('data_2_2_2.txt', 'r') as file:
    M, N, K = [int(x) for x in file.readline().split()]

    q = [0]*(2*(M+N)+2*K+1)
    for index, X in enumerate(file.readline().split()):
        q[index+M+1] = int(X)

    Q = [0] * (K+1)
    for index, X in enumerate(file.readline().split()):
        Q[index + 1] = int(X)

    d = [[int(i) for i in file.readline().split()] for j in range(2*(M+N)+1)]


def expand_start(d):
    n = 2*(M+N)
    n_expand = 2*(M+N)+2*K
    d_expand = np.zeros((n_expand+1, n_expand+1), dtype=int)
    for i in range(1, n+1):
        for j in range(1, n+1):
            d_expand[i][j] = d[i][j]

    for i in range(n+1, n_expand+1):
        for j in range(1, n+1):
            d_expand[i][j] = d[0][j]

    for i in range(1, n+1):
        for j in range(n+1, n_expand+1):
            d_expand[i][j] = d[i][0]

    return d_expand


d = expand_start(d)
# print(M, N, K)
# print(q)
# print(Q)
# for row in d:
#     print(row)

solver = pywraplp.Solver.CreateSolver('SCIP')
INF = 10000

B = {i for i in range(1, 2*(M+N)+2*K+1)}
F1 = set()
F2 = set()
F3 = set()
F4 = set()
F5 = set()
B_2d = set()
for i in B:
    for k in range(1, K+1):
        F1.add((i, k+2*(M+N)))
        F2.add((k+K+2*(M+N), i))
    F3.add((i, i))

for i in range(M+N+1, 2*(M+N)+1):
    for k in range(1, K+1):
        F4.add((k+2*(M+N), i))

for i in range(1, (M+N)+1):
    for k in range(1, K+1):
        F5.add((i, k+K+2*(M+N)))

for i in B:
    for j in B:
        B_2d.add((i, j))
A = B_2d - F1 - F2 - F3 - F4 - F5
# for (i, j) in A:
#     print((i, j))

Ap = [[] for i in range(len(A))]
Am = [[] for i in range(len(A))]
for (i, j) in A:
    Ap[i].append(j)
    Am[j].append(i)

X = {}
L = {}
P = {}
W = {}
Z = {}
for k in range(1, K+1):
    for (i, j) in A:
        X[k, i, j] = solver.IntVar(0, 1, 'X({},{},{})'.format(k, i, j))
    for i in B:
        L[k, i] = solver.IntVar(0, INF, 'L({},{})'.format(k, i))
        P[k, i] = solver.IntVar(0, INF, 'P({},{})'.format(k, i))
        W[k, i] = solver.IntVar(0, INF, 'W({},{})'.format(k, i))
        Z[i] = solver.IntVar(1, K, 'Z({})'.format(i))

# leftVar = param when X[k,i,j] = 1
def ConditionalX_1(leftVar, param, _k, _i, _j):
    c = solver.Constraint(-INF+int(param), INF)
    c.SetCoefficient(X[_k, _i, _j], -INF)
    c.SetCoefficient(leftVar, 1)

    c = solver.Constraint(-INF-int(param), INF)
    c.SetCoefficient(X[_k, _i, _j], -INF)
    c.SetCoefficient(leftVar, -1)

# leftVar = rightVar + param when X[k,i,j] = 1
def ConditionalX_2(leftVar, rightVar, param, _k, _i, _j):
    c = solver.Constraint(-INF+int(param), INF)
    c.SetCoefficient(X[_k, _i, _j], -INF)
    c.SetCoefficient(leftVar, 1)
    c.SetCoefficient(rightVar, -1)

    c = solver.Constraint(-INF-int(param), INF)
    c.SetCoefficient(X[_k, _i, _j], -INF)
    c.SetCoefficient(leftVar, -1)
    c.SetCoefficient(rightVar, 1)


# Constraints
# for i in Ap[8]:
#     print(i)
# Cân bằng luồng
for i in range(1, 2*(M+N)+1):
    c = solver.Constraint(1, 1)
    for k in range(1, K+1):
        for j in Ap[i]:
            c.SetCoefficient(X[k, i, j], 1)

    c = solver.Constraint(1, 1)
    for k in range(1, K+1):
        for j in Am[i]:
            c.SetCoefficient(X[k, j, i], 1)

for i in range(1, 2*(M+N)+1):
    for k in range(1, K+1):
        c = solver.Constraint(0, 0)
        for j in Ap[i]:
            c.SetCoefficient(X[k, i, j], 1)
        for j in Am[i]:
            c.SetCoefficient(X[k, j, i], -1)

# for k in range(1, K+1):
#     c = solver.Constraint(1, 1)
#     for i in Ap[k+2*(M+N)]:
#         c.SetCoefficient(X[k, k+2*(M+N), i], 1)

#     c = solver.Constraint(1, 1)
#     for i in Am[k+K+2*(M+N)]:
#         c.SetCoefficient(X[k, i, k+K+2*(M+N)], 1)


# Cùng tuyến
for k in range(1, K+1):
    for (i, j) in A:
        ConditionalX_2(Z[j], Z[i], 0, k, i, j)

# Nhận trả hàng
for k in range(1, K+1):
    for (i, j) in A:
        ConditionalX_2(Z[j], Z[i], 0, k, i, j)
        ConditionalX_2(L[k, j], L[k, i], d[i][j], k, i, j)
        if j <= M:
            ConditionalX_2(P[k, j], P[k, i], 1, k, i, j)
        elif j <= M+N:
            ConditionalX_2(W[k, j], W[k, i], q[j], k, i, j)
        elif j <= M+N+M:
            ConditionalX_2(P[k, j], P[k, i], -1, k, i, j)
        elif j <= 2*(M+N):
            ConditionalX_2(W[k, j], W[k, i], -q[j - M - N], k, i, j)

# Điều kiện biến
for k in range(1,K+1):
    for i in B:
        c = solver.Constraint(0, 1)
        c.SetCoefficient(P[k,i], 1)

        c = solver.Constraint(0, Q[k])
        c.SetCoefficient(W[k,i], 1)

# Xuất phát
for k in range(1, K+1):
    c = solver.Constraint(0, 0)
    c.SetCoefficient(P[k, k+2*(M+N)], 1)

for k in range(1, K+1):
    c = solver.Constraint(0, 0)
    c.SetCoefficient(W[k, k+2*(M+N)], 1)

# Kết thúc
for k in range(1, K+1):
    for i in Am[k+K+2*(M+N)]:
        ConditionalX_1(P[k, i], 0, k, i, k+K+2*(M+N))
        ConditionalX_1(W[k, i], 0, k, i, k+K+2*(M+N))

# Điều kiện trả hàng/người
for k in range(1, K+1):
    for i in range(1, M+N+1):
        c = solver.Constraint(1, INF)
        c.SetCoefficient(L[k, i], -1)
        c.SetCoefficient(L[k, i+(M+N)], 1)

# Cùng tuyến
for i in range(1, M+N+1):
    c = solver.Constraint(0, 0)
    c.SetCoefficient(Z[i], -1)
    c.SetCoefficient(Z[i+(M+N)], 1)

for k in range(1, K+1):
    c = solver.Constraint(k, k)
    c.SetCoefficient(Z[k+2*(M+N)], 1)

    c = solver.Constraint(k, k)
    c.SetCoefficient(Z[k+K+2*(M+N)], 1)

# Objective
obj = solver.Objective()
for k in range(1, K+1):
    for (i, j) in A:
        obj.SetCoefficient(X[k, i, j], int(d[i][j]))

obj.SetMinimization()

result_status = solver.Solve()
assert result_status == pywraplp.Solver.OPTIMAL

print('optimal objective value: %.2f' % solver.Objective().Value())
    

# print route
for k in range(1, K+1):
    rs = [None] * (2*(N+M+K))
    for (i, j) in A:
        if X[k, i, j].solution_value() > 0:
            print(i, j, P[k, j].solution_value(), W[k, j].solution_value(), L[k, j].solution_value())
            rs[i] = j
    

    print(rs)
    # index = k + N + M - 1
    # while(index != k + K + (N+M)):
    #     print(rs[index])
    #     index = rs[index]
