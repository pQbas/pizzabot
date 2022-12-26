import numpy as np

x = np.zeros([2,1])
p = np.zeros([2,1])

x[0] = 60
p[0] = 225

i = 0

while i<=10:
    x[1] = x[0]
    p[1] = p[0]

    z = 60 + np.random.normal(0,5)
    r = 25

    k = p[1]/(p[1] + r)

    x[0] = x[1] + k*(z - x[1])
    p[0] = (1 - k)*p[1]

    print(z)
    print(x[0],p[0])
    
    i+=1


