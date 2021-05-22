import numpy as np
import time
from ortools.sat.python import cp_model

start = time.time()
with open('data.txt', 'r') as file:
    m,n,k = [int(x) for x in file.readline().split()]
    q = [int(x) for x in file.readline().split()]
    Q = [int(x) for x in file.readline().split()]
    d = [[int(i) for i in file.readline().split()] for j in range(2*m+2*n+1)]

# from pprint import pprint
# m=3
# n=4
# k=2
# # pprint(q)
# # pprint(Q)
# q=[4,3,5,6]
# Q=[10,10]


# d=[[0, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 2, 2, 9],
#  [3, 0, 5, 2, 4, 2, 3, 3, 4, 3, 5, 6, 7, 2, 9],
#  [3, 4, 0, 2, 1, 4, 3, 3, 4, 3, 5, 6, 1, 2, 9],
#  [3, 4, 5, 0, 4, 2, 2, 3, 4, 3, 5, 6, 7, 3, 8],
#  [3, 7, 1, 2, 0, 4, 3, 3, 4, 2, 5, 5, 7, 2, 9],
#  [3, 4, 5, 2, 4, 0, 3, 6, 4, 3, 5, 4, 7, 2, 4],
#  [3, 4, 1, 2, 4, 4, 0, 3, 4, 3, 5, 6, 7, 2, 9],
#  [3, 3, 5, 3, 7, 1, 3, 0, 4, 8, 5, 5, 4, 2, 3],
#  [3, 4, 2, 2, 4, 4, 5, 3, 0, 3, 5, 6, 7, 2, 9],
#  [1, 4, 5, 4, 4, 1, 3, 1, 4, 0, 5, 9, 8, 2, 3],
#  [3, 4, 2, 2, 5, 4, 3, 3, 4, 3, 0, 6, 7, 2, 9],
#  [2, 5, 5, 1, 4, 5, 7, 1, 4, 3, 5, 0, 7, 2, 3],
#  [3, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 0, 2, 9],
#  [2, 4, 5, 1, 6, 8, 3, 8, 4, 3, 5, 6, 6, 0, 9],
#  [3, 4, 5, 2, 4, 4, 3, 3, 4, 3, 5, 6, 7, 2, 0]]


# pprint(q)
# pprint(Q)
# pprint(d)

N= 2*m+2*n
K=k
# bỏ hết hàng đầu cột đầu thay bằng 0
# thêm 2k điểm logic tham chiếu tới điểm 0 vào cuối 

    
c = [[0 for i in range(N+2*K)] for j in range(N+2*K)]
for i in range(N):
    for j in range(N):
        c[i][j] = d[i+1][j+1]
from pprint import pprint

# pprint(c)
for i in range(N, N+2*K):
    for j in range(N):
        if j< N:
            c[i][j] = d[0][j+1]
            c[j][i] = d[j+1][0]


# pprint(c)



    
model = cp_model.CpModel()
# modeling
# VAR

x = [model.NewIntVar(0, N+2*K-1, f'x[{i}]') for i in range(N+K)]
IR = [model.NewIntVar(0, K-1, f'IR[{i}]') for i in range(N+2*K)]

M=np.sum(c)

l = [model.NewIntVar(0, int(M), f'l[{i}]') for i in range(N+2*K)]
# y = model.NewIntVar(0, int(M), 'y')

w = [model.NewIntVar(0, max(Q), 'w[{}]'.format(i)) for i in range(N+2*K)]   # weight
p = [model.NewBoolVar('p[{}]'.format(i)) for i in range(N+2*K)]  

# x[i]= j : điểm tiếp theo của i là j 
# x = [model.NewIntVar(1,num_point+1,'x[{}]'.format(i)) for i in range(num_point+1)]
# # khoảng cách tích lũy 
# l = [model.NewIntVar(0, 1000, 'l[{}]'.format(i)) for i in range(nm2+2*k+2)]
# w = [model.NewIntVar(0, max(Q), 'w[{}]'.format(i)) for i in range(nm2+2*k+2)]   # weight
# p = [model.NewBoolVar('p[{}]'.format(i)) for i in range(nm2+2*k+2)]        # person
# index_router = [model.NewIntVar(1, k, 'index_router[{}]'.format(i)) for i in range(nm2+2*k+2)]  # index_router which car k
obj = model.NewIntVar(0, 10000, 'obj')


accum=0
for i in range(N):
    accum= accum+i 

for i in range(N+K,N+2*K):
    accum=accum+i
model.Add(sum([x[i] for i in range(N+K)]) ==accum )

# for i in range(N+K,N+2*K):
#     model.Add(x[i]==i)

for i in range(N, N+K):
    model.Add(l[i]==0)
    model.Add(w[i] == 0)
    model.Add(p[i] == 0)


model.AddAllDifferent(x)

for i in range(K):
    model.Add(IR[N+i]==IR[N+K+i])

for i in range(N+K):
    model.AddAllDifferent([x[i], i])

# for s in range(1, nm2+2*k+1):    # khoi tao cho tat ca cac diem 
    # model.Add(l[s] == 0)
    # model.Add(w[s] == 0)
    # model.Add(p[s] == 0)
# for s in range(nm2+1, nm2+k+1):   # for k 
#     model.Add(index_router[s] == s-nm2)      # điểm đầu và điểm cuối cùng  tuyen 1..k
#     model.Add(index_router[s+k] == s-nm2)

# diem 0 thi k xet
# for i in range(0,nm2+2*k+1):
#     model.Add(x[0] == 0)
# # khogn quay lai chinh no 
# for i in range(1,nm2+2*k+1):
#     model.Add(x1[i][i] == 0)
# chi vao va ra tai 1 tiem 
# for i in range(1,nm2+1):
#     model.Add(sum([x1[i][j] for j in range(1,nm2+1)]) == 1)
# bien 
# for i in range(1,k+1):
#     model.Add(sum([x1[nm2+i][j] for j in range(1,nm2+1)]) == 1)   # chi ra 1 diem 

# for i in range(1,k+1):
#     model.Add(sum([x1[j][i+nm2+k] for j in range(1,nm2+1)])== 1)   # chi toi 1 diem 
        
# dest = [i for i in range(1,nm2+1)]
# for i in range(k):
#     dest.append(nm2+k+i+1)
# for i in dest:
#     model.Add(sum([x1[j][i] for j in range(1,nm2+k+1)]) == 1)
# để tăng tốc
# for i in range(nm2+1, nm2+k+1):
#     for j in range(n+m+1, N-k):
#         model.Add(x1[i][j] == 0)
#     for j in range(1, nm2+k+1):
#         model.Add(x1[j][i] == 0)

b=[[model.NewBoolVar(f'b[{i}][{j}]') for j in range(N+2*K)] for i in range(N+K)]
for i in range(N+K):
    for j in range(N+2*K):
        model.Add(x[i]==j).OnlyEnforceIf(b[i][j])
        model.Add(x[i]!=j).OnlyEnforceIf(b[i][j].Not())
        model.Add(IR[i]==IR[j]).OnlyEnforceIf(b[i][j])
        model.Add(l[j]-l[i]==c[i][j]).OnlyEnforceIf(b[i][j])
        if j < m:
            model.Add(p[j] == p[i]+1).OnlyEnforceIf(b[i][j])
            model.Add(w[j] == w[i]).OnlyEnforceIf(b[i][j])
        elif j < m+n:
            model.Add(p[j] == p[i]).OnlyEnforceIf(b[i][j])
            model.Add(w[j] == w[i] + q[j-n]).OnlyEnforceIf(b[i][j])
        elif j < m+n+m:
            model.Add(p[j] == p[i]-1).OnlyEnforceIf(b[i][j])
            model.Add(w[j] == w[i]).OnlyEnforceIf(b[i][j])
        elif j < m+n+m+n:
            model.Add(p[j] == p[i]).OnlyEnforceIf(b[i][j])
            model.Add(w[j] == w[i] - q[j-m-n-n]).OnlyEnforceIf(b[i][j])


# khong  Uy ve diem dau TIEN 
for i in range(N+K):
    for j in range(N, N+K):
        model.AddAllDifferent([x[i], j])

# các ĐK khi có đường từ i -> j
# for i in range(1,nm2+2*k+1): 
#     for j in range(1,nm2+2*k+1):

#         model.Add(index_router[j] == index_router[i]).OnlyEnforceIf(x1[i][j])
#         model.Add(l[j] == l[i] + D[i][j]).OnlyEnforceIf(x1[i][j])
#         # don khach
#         if j <= n:
#             model.Add(p[j] == p[i]+1).OnlyEnforceIf(x1[i][j])
#             model.Add(w[j] == w[i]).OnlyEnforceIf(x1[i][j])
#         elif j <= n+m:
#             model.Add(p[j] == p[i]).OnlyEnforceIf(x1[i][j])
#             model.Add(w[j] == w[i] + q[j-n]).OnlyEnforceIf(x1[i][j])
#         elif j <= n+m+n:
#             model.Add(p[j] == p[i]-1).OnlyEnforceIf(x1[i][j])
#             model.Add(w[j] == w[i]).OnlyEnforceIf(x1[i][j])
#         elif j <= nm2:
#             model.Add(p[j] == p[i]).OnlyEnforceIf(x1[i][j])
#             model.Add(w[j] == w[i] - q[j-n-m-n]).OnlyEnforceIf(x1[i][j])

# # ĐK trả hàng đúng tuyến
for i in range(m+n):
    model.Add(IR[i] == IR[i+m+n])

# # ĐK trả đến sau
for i in range(n+m):
    model.Add(l[i] < l[i+n+m])
        
# # ĐK trọng tải max
for i in range(N+K):
    model.Add(w[i] <= 10)
    model.Add(0<= w[i] )

# # điều kiện chở người max
for i in range(N+K): 
    model.Add(p[i] <= 1) 
    model.Add(0<= p[i] )
    # model.Add(p[i+n+m] == 0)

# truong hợp tổng quát max w < Q
# for i in range(n+k+1):
    # for j in range(1, k+1):
    #     model.Add(b_index_router[j][i] == index_router[i]==j)
    #     model.Add(w[i] <= Q[j-1]).OnlyEnforceIf(b_index_router[j][i])
    

model.Add(obj==sum([l[i] for i in range(N+K, N+2*K)]))
model.Minimize(obj)


solver = cp_model.CpSolver()
status = solver.Solve(model)
if status == cp_model.OPTIMAL:
    print('objective: ', solver.ObjectiveValue())
    # for i in range(N+k):
    #     print(solver.Value(i), end=' ')
    # print()
    # for i in range(N+k):
    #     print(solver.Value(x[i]), end=' ')
    # print()
    # for i in range(N+k):
    #     for j in range(1,N):
    #         print(solver.Value(x1[i][j]), end=' ')
    #     print()

    # for l in range(1, k+1):
    #     for i in range(1, nm2+2*k+1):
    #         for j in range(1,nm2+2*k+1):
    #             if solver.Value(x1[i][j]) == 1 and solver.Value(index_router[j]) == l:
    #                 print('{} -- {}'.format(i,j))
    #     print()
    for i in range(N, N+K):
        j=i
        print(f'xe {j-N+1}: {i}   ', end='')
        while j<N+K:
            print(solver.Value(x[j]),  '   ', end='')
            j=solver.Value(x[j])
        print('\n')
else :
    print('not found')
print('time:', time.time() - start)
