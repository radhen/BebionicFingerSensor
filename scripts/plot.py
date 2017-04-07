import matplotlib.pyplot as plt
import os
import scipy.stats
import numpy as np
import scipy.signal as signal

os.getcwd()
# os.path.exists()
# print (os.getcwd())

# data plotting
for j in range(4):
    file = open("../calibTest_1_15-3/forceData15-3/test{}_Al.txt".format(j+1), "r")
    data = file.read()
    # print (type(data))

    a = []
    b = []
    # print b
    for i in range(len(data.split("\n"))-1):
        ints = [int(x) for x in data.split("\n")[i].split()]
        # print (data.split("\n")[i].split())
        # print (ints[0]+ints[1])
        if (i==0):
            baseValue = ints[0]+ints[1]
        # a.append((ints[0]+ints[1])) # plotting raw values
        a.append((ints[0]+ints[1]) - baseValue) # plotting raw values
        b.append(i)


    # print (a)
    # print ("base value is: " + str(baseValue))

    # ===== Calculating slopes ===== #
    # del b[-1]
    # slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(a,b)
    # print abs(slope)
    # print np.degrees(np.arctan(slope)) #print slope in degrees

    # ===== plotting ===== #
    plt.plot(b,np.absolute(a),linewidth = 0.6, label = "test{}_Cu".format(j+1))
    plt.hold(True)

plt.legend()
plt.show()
