import matplotlib.pyplot as plt
import os
import scipy.stats
import numpy as np
import scipy.signal as signal

os.getcwd()
# os.path.exists()
# print (os.getcwd())

# data plotting
for j in range(5):
    file = open("../calibTest_2_13-4/forceData13-4/20mA/test{}_Cu.txt".format(j+1), "r")
    data = file.read()
    # print (type(data))

    a = []
    b = []
    # print b
    for i in range(len(data.split("\n"))-1):
        ints = [int(x) for x in data.split("\n")[i].split()]
        # print (data.split("\n")[i].split())
        # print (ints[2])

        # if (i==0):
        #     baseValue = ints[2]+ints[3]
        # a.append((ints[2]+ints[3]) - baseValue) # plotting raw values

        a.append((ints[2]+ints[3])) # plotting raw values

        b.append(i)



    print (len(a))
    # print ("base value is: " + str(baseValue))

    # ===== Calculating slopes ===== #
    # del b[-1]
    # slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(a,b)
    # print abs(slope)
    # print np.degrees(np.arctan(slope)) #print slope in degrees

    # ===== plotting ===== #
    plt.plot(b,a,linewidth = 0.6, label = "test{}_Cu".format(j+1))
    plt.hold(True)

plt.legend()
plt.show()
