from scipy import signal
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import LabelEncoder


angles = [0]
maxForces = [5]
d = {}
# df_empty = pd.DataFrame({'IR' : [], 'baro' : [], 'force' : [], 'degree' : []})
df_empty = pd.DataFrame()
for angle in angles:
    for maxForce in maxForces:
        df = pd.read_excel('{}deg_{}N.xls'.format(angle, maxForce))
        data=df.as_matrix() #converting into numpy array
        ## DATAFIELDSrow_1=IR, row_2=baro, row_3=force, row_4=deg

        ## finding the first occurence of nan to cal length of IR/Force (16 bit)
        len_sensorData = np.where(np.isnan(data[:,2]))[0][1] - 1
        ## resampling MTSdata to match len of IR/baro
        f = signal.resample(data[439:18130,3],2755)
        # print (len_sensorData)
        # print (f.size)

        # df_tmp = {'IR':df['IR (16 bit)'].values[:len_sensorData],'baro':df['Force (16 bit)'].values[:len_sensorData],'force':f, 'degree':df['Degree'].values[:len_sensorData]}
        # data_1 = pd.DataFrame(df_tmp)
        # df_empty = pd.concat([df_empty,data_1])

        # plotting stuff
        fig, ax1 = plt.subplots()
        plt.Figure(figsize=(10,5))
        ax1.plot(data[223:2978,2], 'b-',markersize=0.5)
        ax1.set_ylabel('Force16bit', color='b')
        ax1.tick_params('y', colors='b')

        ax2 = ax1.twinx()
        ax2.plot(f, 'r-',markersize=0.5)
        ax2.set_ylabel('Force(N)', color='r')
        ax2.tick_params('y', colors='r')

        plt.title('{}degree {}N maxF'.format(angle, maxForce))
        fig.tight_layout()
        plt.show()



# print (df_empty)

# lable_encoder = LabelEncoder().fit(df_empty.degree)
# lables = lable_encoder.transform(df_empty.degree)
# print (lable_encoder.classes_)
