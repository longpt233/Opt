import numpy as np
import time
from ortools.sat.python import cp_model


# Có N hành khách 1, 2, …, N và M gói hàng N+1,N+ 2, …, N+M. Hành khách (người) (hoặc gói hàng) i có điểm đón là i và điểm trả là i+ N + M (i = 1,2,…,2N+2M). 
# Mỗi gói hàng i có khối lượng qi (i=N+1,…,N+M)
# Có K taxi 1,…,K xuất phát từ điểm 0. Mỗi xe taxi k có thể vận chuyển cùng 1 lúc 1 hành khách và tối đa Qk khối lượng hàng (xếp vào cốp xe) (k=1,…,K)
# Biết rằng d(i,j) là khoảng cách từ điểm i đến điểm j (I,j=0,…,2N+2M)
# Hãy tính toán phương án vận chuyển sao cho tổng quãng đường di chuyển của các xe ngắn nhất

start = time.time()
with open('project/data.txt', 'r') as file:
    m,n,k = [int(x) for x in file.readline().split()]
    q = [int(x) for x in file.readline().split()]
    Q = [int(x) for x in file.readline().split()]
    d = [[int(i) for i in file.readline().split()] for j in range(2*m+2*n+1)]
    nm2 = 2*m+2*n
    
def expend_start(number, d):
    n_expend = len(d) + k*2
    
    newd = np.zeros((n_expend, n_expend), dtype=int)
    for i in range(1,number+1):
        for j in range(1,number+1):
            newd[i][j] = d[i][j]
    
    for i in range(number+1, n_expend):
        for j in range(1, number+1):        
            newd[i][j] = d[0][j]
            
    for i in range(1, number+1):
        for j in range(number+1, n_expend):        
            newd[i][j] = d[i][0]
        
    return n_expend, newd

N, D = expend_start(nm2, d)
q.insert(0,0)

# small test
n = 3
m = 3
nm2 = 2*n+2*m
N = nm2+ 2*k +1
    
model = cp_model.CpModel()
x1 = [[model.NewBoolVar('x1[{},{}]'.format(j,i)) for i in range(N)] for j in range(nm2+k+1)] # x[i][i] == 1 <=> có đường từ i ->j

l = [model.NewIntVar(0, 1000, 'l[{}]'.format(i)) for i in range(N)]
w = [model.NewIntVar(0, max(Q), 'w[{}]'.format(i)) for i in range(N)]   # weight
max_w = [model.NewIntVar(0, max(Q), 'max_w[{}]'.format(i)) for i in range(N)]        # max weight
p = [model.NewBoolVar('p[{}]'.format(i)) for i in range(N)]        # person
mark = [model.NewIntVar(1, k, 'mark[{}]'.format(i)) for i in range(N)]  # mark which car k
obj = model.NewIntVar(0, 10000, 'obj')

for s in range(nm2+1, nm2+k+1):
    model.Add(l[s] == 0)
    model.Add(w[s] == 0)
    model.Add(p[s] == 0)
    model.Add(max_w[s] == Q[s-nm2-1])
    # model.Add(p[s+k] == 0)
    model.Add(mark[s] == s-nm2)   # điểm đầu và điểm cuối cùng tuyến
    model.Add(mark[s+k] == s-nm2)

# điều kiện chở người
for i in range(1, n+1): 
    model.Add(p[i] == 1) 
    model.Add(p[i+n+m] == 0)
    
# DK 1 đường đến i và 1 đường từ i
for i in range(1,nm2+k+1):
    model.Add(x1[i][i] == 0)
    model.Add(sum([x1[i][j] for j in range(1,N)]) == 1)
        
dest = [i for i in range(1,nm2+1)]
for i in range(k):
    dest.append(nm2+k+i+1)
for i in dest:
    model.Add(sum([x1[j][i] for j in range(1,nm2+k+1)]) == 1)
# để tăng tốc
for i in range(nm2+1, nm2+k+1):
    for j in range(n+m+1, N-k):
        model.Add(x1[i][j] == 0)
    for j in range(1, nm2+k+1):
        model.Add(x1[j][i] == 0)

# các ĐK khi có đường từ i -> j
for i in range(1,nm2+k+1): 
    for j in range(1,N):
        model.Add(mark[j] == mark[i]).OnlyEnforceIf(x1[i][j])
        model.Add(max_w[j] == max_w[i]).OnlyEnforceIf(x1[i][j])
        model.Add(l[j] == l[i] + D[i][j]).OnlyEnforceIf(x1[i][j])
        if j <= n:
            model.Add(p[j] == p[i]+1).OnlyEnforceIf(x1[i][j])
            model.Add(w[j] == w[i]).OnlyEnforceIf(x1[i][j])
        elif j <= n+m:
            model.Add(p[j] == p[i]).OnlyEnforceIf(x1[i][j])
            model.Add(w[j] == w[i] + q[j-n]).OnlyEnforceIf(x1[i][j])
        elif j <= n+m+n:
            model.Add(p[j] == p[i]-1).OnlyEnforceIf(x1[i][j])
            model.Add(w[j] == w[i]).OnlyEnforceIf(x1[i][j])
        elif j <= nm2:
            model.Add(p[j] == p[i]).OnlyEnforceIf(x1[i][j])
            model.Add(w[j] == w[i] - q[j-n-m-n]).OnlyEnforceIf(x1[i][j])

# ĐK trả hàng đúng tuyến
for i in range(1, n+m+1):
    model.Add(mark[i] == mark[i+m+n])

# ĐK trả đến sau
for i in range(n+m+1, nm2+1):
    model.Add(l[i] > l[i-n-m])
        
# ĐK trọng tải max
for i in range(1,nm2+1):
    model.Add(w[i] <= max_w[i])
    

model.Add(obj==sum([l[i] for i in range(nm2+k+1, N)]))
model.Minimize(obj)

solver = cp_model.CpSolver()
status = solver.Solve(model)


if status == cp_model.OPTIMAL:
    print('objective: ', solver.ObjectiveValue())
    # for i in range(nm2+k+1):
    #     print(solver.Value(i), end=' ')
    # print()
    # for i in range(nm2+k+1):
    #     print(solver.Value(x[i]), end=' ')
    # print()
    # for i in range(1, nm2+k+1):
    #     for j in range(1,N):
    #         print(solver.Value(x1[i][j]), end=' ')
    #     print()

    for l in range(1, k+1):
        for i in range(1, nm2+k+1):
            for j in range(1,N):
                if solver.Value(x1[i][j]) == 1 and solver.Value(mark[j]) == l:
                    print('{} -- {}'.format(i,j))
        print()
else :
    print('not found')
print('time:', time.time() - start)
