import matplotlib.pyplot as plt
import os

os.getcwd()
# os.path.exists()
# print (os.getcwd())

# for j in range(7):
#     file = open("../calibTest_1_15-3/forceData15-3/test{}_Cu.txt".format(j+1), "r")
#     data = file.read()
#     # print (type(data))
#
#     a = []
#
#     for i in range(len(data.split("\n"))-1):
#         ints = [int(x) for x in data.split("\n")[i].split()]
#         # print (data.split("\n")[i].split())
#         # print (ints[0]+ints[1])
#         a.append((ints[0]+ints[1]))
#     print (max(a)-min(a))

## data plotting
for j in range(7):
    file = open("../calibTest_1_15-3/forceData15-3/test{}_Cu.txt".format(j+1), "r")
    data = file.read()
    # print (type(data))

    a = []
    b = []
    for i in range(len(data.split("\n"))-1):
        ints = [int(x) for x in data.split("\n")[i].split()]
        # print (data.split("\n")[i].split())
        # print (ints[0]+ints[1])
        a.append((ints[0]+ints[1]))
    # print len(a)
    print ("max - min is: "+str(max(a)-min(a)))
    leg = plt.plot(a,'.',linewidth = 0.25, label = "test{}_Cu.txt".format(j+1))
    plt.hold(True)
    b.append(leg)


print len(b)
# plt.legend(handles=[b])
# plt.show()
