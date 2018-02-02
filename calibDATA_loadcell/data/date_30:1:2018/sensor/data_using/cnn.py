from scipy import signal
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import LabelEncoder


angles = [0,20,-20]
maxForces = [50,5,1]
d = {}
# df_empty = pd.DataFrame({'IR' : [], 'baro' : [], 'force' : [], 'degree' : []})
df_empty = pd.DataFrame()
for angle in angles:
    for maxForce in maxForces:
        df = pd.read_excel('{}deg_{}N.xls'.format(angle, maxForce),sheet_name='Sheet1')
        data=df.as_matrix() #converting into numpy array
        ## row_1=IR, row_2=baro, row_3=force, row_4=deg

        ## finding the first occurence of nan to cal length of IR/Force (16 bit)
        len_sensorData = np.where(np.isnan(data[:,2]))[0][1] - 1
        ## resampling MTSdata to match len of IR/baro
        f = signal.resample(data[:,3],len_sensorData)

        df_tmp = {'IR':df['IR (16 bit)'].values[:len_sensorData],'baro':df['Force (16 bit)'].values[:len_sensorData],'force':f, 'degree':df['Degree'].values[:len_sensorData]}
        data_1 = pd.DataFrame(df_tmp)
        df_empty = pd.concat([df_empty,data_1])

        ## plotting stuff
        # plt.figure(figsize=(10,5))
        # plt.plot(data[:len_sensorData,2])
        # plt.title('{}degree {}N maxF'.format(angle, maxForce))
        # plt.show()

# print (df_empty['IR'].values[0])

lable_encoder = LabelEncoder().fit(df_empty.degree)
# print (lables)
lables = lable_encoder.transform(df_empty.degree)
print (lable_encoder.classes_)
