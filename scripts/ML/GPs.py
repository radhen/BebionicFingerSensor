import numpy as np
import matplotlib.pyplot as pl
import pandas as pd
from sklearn.preprocessing import StandardScaler
import math


def preprocessData(X):

    # ----- normalizing between 0 and 1 ----- #
    X = (X - min(X))/float(max(X)-min(X))

    '''normalizing data to zero mean and 1 std. dev.
    '''
    # series = pd.Series(X[:])
    # # prepare data for normalization
    # values = series.values
    # values = values.reshape((len(values), 1))
    # # train the normalization
    # scaler = StandardScaler()
    # scaler = scaler.fit(values)
    # # print('Mean: %f, StandardDeviation: %f' % (scaler.mean_, math.sqrt(scaler.var_)))
    # # normalize the dataset and print
    # standardized = scaler.transform(values)
    # # print (standardized[:,0])
    # X = standardized[:,0]
    # # print(standardized.shape)
    # # inverse transform and print
    # # inversed = scaler.inverse_transform(standardized)
    # # print(inversed.shape)
    # # X[i,:,0] = inversed[:]
    # # print (type(inversed))
    #
    # # print (X[0,:,0])
    # # print (X[0,:,1])
    return (X)


'''load data'''
df_newton = pd.read_excel('30N_gp.xlsx',sheet_name='Sheet1',header=None)
df_baro = pd.read_excel('30N_gp.xlsx',sheet_name='Sheet2',header=None)
df_ir = pd.read_excel('30N_gp.xlsx',sheet_name='Sheet3',header=None)

data_x = df_baro.values[:151]
data_x1 = df_ir.values[:151]
data_y = df_newton.values[:151]

n = data_x.shape[0]
'''Normalizing test data b/w 0 and 1'''
Xtest = data_x[:,2].reshape(-1,1)
Xtest = preprocessData(Xtest[:,0])
Xtest = Xtest.reshape(-1,1)

l = Xtest.tolist()
l.sort()

Xtest = np.array(l)

'''Define the kernel function'''
def kernel(a, b, param):
    sqdist = np.sum(a**2,1).reshape(-1,1) + np.sum(b**2,1) - 2*np.dot(a, b.T)
    return (0.05*np.exp(-.5 * (1/param) * sqdist))

param = 0.60
K_ss = kernel(Xtest, Xtest, param)

'''Get cholesky decomposition (square root) of the covariance matrix'''
L = np.linalg.cholesky(K_ss + 1e-2*np.eye(n))
'''Sample 3 sets of standard normals for our test points,
multiply them by the square root of the covariance matrix'''
f_prior = np.dot(L, np.random.normal(size=(n,1)))

'''Now let's plot the 3 sampled functions.'''
# pl.plot(Xtest, f_prior)
# pl.axis([-1.5, 1.5, -2, 2])
# pl.title('Three samples from the GP prior')
# pl.show()

''''Normalizing training data b/w 0 and 1'''
Xtrain = data_x[:,0].reshape(-1,1)
Xtrain = preprocessData(Xtrain[:,0])
Xtrain = Xtrain.reshape(-1,1)
ytrain = data_y[:,0].reshape(-1,1)
ytrain = preprocessData(ytrain[:,0])
ytrain = ytrain.reshape(-1,1)

''''Apply the kernel function to our training points'''
K = kernel(Xtrain, Xtrain, param)
L = np.linalg.cholesky(K + 0.00005*np.eye(len(Xtrain)))

'''Compute the mean at our test points'''
K_s = kernel(Xtrain, Xtest, param)
Lk = np.linalg.solve(L, K_s)
mu = np.dot(Lk.T, np.linalg.solve(L, ytrain)).reshape((n,))

'''Compute the standard deviation so we can plot it'''
s2 = np.diag(K_ss) - np.sum(Lk**2, axis=0)
stdv = np.sqrt(s2)
# Draw samples from the posterior at our test points.
L = np.linalg.cholesky(K_ss + 1e-6*np.eye(n) - np.dot(Lk.T, Lk))
f_post = mu.reshape(-1,1) + np.dot(L, np.random.normal(size=(n,1)))

pl.plot(Xtrain, ytrain, 'bs', ms=2)
pl.plot(Xtest, f_post)
pl.gca().fill_between(Xtest.flat, mu-2*stdv, mu+2*stdv, color="#dddddd")
pl.plot(Xtest, mu, 'r--', lw=1)
pl.axis([-0.25, 1.25, -0.2, 1.2])
pl.title('Three samples from the GP posterior')
pl.show()
