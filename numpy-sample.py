import numpy as np
#np.zeros(shape,dtype,order)
a = np.zeros([2,3])
b = np.ones((2,3))
print(b)
c = np.arange(10)
print(c)
# d = np.random(10)
e = np.empty([2,3])  #数值为随机，未初始化


# numpy.array(object, dtype = None, copy = True, 
#				order=None, subok=False,ndmin=0)


student = np.dtype([('name','S20'), ('age', 'i1'), ('marks', 'f4')]) 
dt = np.dtype([('nam','S20')])
a = np.array([2,3,4],dt)
print(a)
print(a['nam'])
print(student)

a = np.arange(24)
print(a.ndim)
b = a.reshape(2,4,3)
print(b)
print(b.ndim)

#*****************************np.asarray****************
x = [1,2,3]
#np.asarray(a,dtype,order)
a= np.asarray(x)
print(a)

#**************************np.frombuffer*************)
#np.frombuffer(buffer,dtype,count,offset)
s=b"hello world"
print(s)
a=np.frombuffer(s,"S1")
print(a)
#************************np.fromiter***************
#np.fromiter(iterable,dtype,count=-1)

list = range(5)
x = np.fromiter(list,float)
print(x)


#************************np.arange****************
#np.arange(start,stop,step,dtype)

x = np.arange(8,0,-2)
print(x)
#*************************np.linspace****************
#np.linspace(start,stop,num=50,endpoint=True,retstep=False,dtype)
#等差数列
a = np.linspace(1,10,10)
print(a)
a = np.linspace(1,10,10,retstep = True)
print(a)
b = np.linspace(1,10,10).reshape(10,1)
print(b)
#**************************np.logspace**************
#np.logspace(start,stop,num,endpoint,base,dtype)
#等比数列
#para base =10 对数底数
a = np.logspace(1,2, num=10)
print(a)
a = np.logspace(0,9,10,base = 2)
print(a)

#**************************slice*********************
a = range(10)
s = slice(2,7,2)
print(a[s])
print(a[2:7:2])

a = np.arange(12).reshape(3,4)
print(a)
print(a[1:])
print(a[:,1])
print(a[1,...])
print(a[...,1:])