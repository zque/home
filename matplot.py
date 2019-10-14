import numpy as np 
from matplotlib import pyplot as plt 
a = np.loadtxt("data-adc11.txt")
#b=a[:,3]
c=a[4000:4100]
y=a
x=range(0,len(y))
plt.title("Matplotlib demo") 
plt.xlabel("x axis caption") 
plt.ylabel("y axis caption") 
plt.plot(x,y) 
plt.show()