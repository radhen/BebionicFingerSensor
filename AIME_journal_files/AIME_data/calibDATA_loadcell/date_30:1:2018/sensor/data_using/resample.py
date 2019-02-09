import pandas as pd
import numpy as np
from scipy import signal

df = pd.read_excel('0deg_50N.xls')
values = df.values
# print (values[:,1].shape)

resampled = signal.resample(values[:,3],3842)
# print (resampled)
np.savetxt('test',resampled)
