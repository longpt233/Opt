import random
import math

# n,m,k
n=3
m=4  
K=2
# const
init_random=10
min_q=4        # min max mỗi điểm nhận/ trả hàng 
max_q=6
min_Q=10       # min max tải trọng xe 
max_Q=10


# bị 3 điểm thẳng hàng
def distance(i1, j1, i2, j2):
	return int(math.sqrt((i1-i2)*(i1-i2)+(j1-j2)*(j1-j2)))+1

# sinh các điểm trên vòng tròn để không có 3 điểm nào thẳng hàng
point=[[init_random*math.cos(i*2*math.pi/(2*n+1)), init_random*math.sin(i*2*math.pi/(2*n+1))] for i in range(2*n+1)] 
# print(point)
# đảo ngẫu nhiên vị trí các điểm trong dãy điểm
for i in range(n*2):
	tmp=random.randint(0, len(point)-1)
	remove=point[tmp]
	point.remove(point[tmp])
	point.append(remove)

# print(point)

c=[[0 for i in range(2*n+1)] for j in range(2*n+1)]
for i in range(2*n+1):
	for j in range(2*n+1):
		if j!=i:
			c[i][j]=distance(point[i][0], point[i][1], point[j][0], point[j][1])
# print(c)
q=[random.randint(min_q, max_q) for i in range(m) ]
Q=[random.randint(min_Q, max_Q) for i in range(K) ]
with open('data_'+str(n)+'_'+str(m)+'_'+str(K)+'.txt', 'w') as f:
	f.writelines(str(n)+ ' '+str(m)+ ' '+str(K)+' '+'\n')
	for i in range(K):
		f.write(str(q[i])+' ')
	f.write('\n')
	for i in range(K):
		f.write(str(Q[i])+' ')
	f.write('\n')
	for i in range(2*n+1):
		for j in range(2*n+1):
			f.write(str(c[i][j])+' ')
		f.write('\n')
