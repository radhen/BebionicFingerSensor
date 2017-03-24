import matplotlib.pyplot as plt
import os
import scipy.stats
import numpy as np
import scipy.signal as signal

os.getcwd()
# os.path.exists()
# print (os.getcwd())

# data plotting
for j in range(7):
    file = open("../calibTest_1_15-3/forceData15-3/test{}_Cu.txt".format(j+1), "r")
    data = file.read()
    # print (type(data))

    a = []
    b = []
    # print b
    for i in range(len(data.split("\n"))-1):
        ints = [int(x) for x in data.split("\n")[i].split()]
        # print (data.split("\n")[i].split())
        # print (ints[0]+ints[1])
        a.append((ints[0]+ints[1]))
        b.append(i)
    # print len(a)
    # print (b)
    # yconst = signal.detrend(a, type='linear')
    slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(a,b)
    # print abs(slope)
    print np.degrees(np.arctan(slope)) #print slope in degrees
    # print ("max - min is: "+str(max(a)-min(a)))
    plt.plot(a,'.',linewidth = 0.25, label = "test{}_Cu".format(j+1))
    # plt.plot(yconst,linewidth = 0.25, label = "test{}_Cu".format(j+1))
    plt.hold(True)

plt.legend()
plt.show()
