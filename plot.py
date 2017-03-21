import matplotlib.pyplot as plt
import os

os.getcwd()
# os.path.exists()
# print (os.getcwd())
for j in range(4):
    file = open((str(os.getcwd())+"/forceData15-3/test{}_Al.txt".format(j+1)), "r")
    data = file.read()
    # print (type(data))

    a = []

    for i in range(len(data.split("\n"))-1):
        ints = [int(x) for x in data.split("\n")[i].split()]
        # print (data.split("\n")[i].split())
        # print (ints[0]+ints[1])
        a.append((ints[0]+ints[1]))
    print len(a)
    plt.plot(a,'.',linewidth = 0.25)
    plt.hold(True)

plt.show()
