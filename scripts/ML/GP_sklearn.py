import numpy as np
import matplotlib.pyplot as pl
import pandas as pd
from sklearn.preprocessing import StandardScaler
import math

import numpy as np
from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C




def preprocessData(X):
    '''normalizing between 0 and 1'''
    X = (X - min(X))/float(max(X)-min(X))
    return (X)

def sortData(A):
    l = A.tolist()
    l.sort()  # sorting helped in removing the sudden jumps.
    return (np.array(l))


def load_data():
    '''load data'''
    df_newton = pd.read_excel('30N_gp.xlsx',sheetname='Sheet1',header=None)
    df_baro = pd.read_excel('30N_gp.xlsx',sheetname='Sheet2',header=None)
    df_ir = pd.read_excel('30N_gp.xlsx',sheetname='Sheet3',header=None)

    data_x = df_baro.values[:151]
    data_x1 = df_ir.values[:151]
    data_y = df_newton.values[:151]

    return (data_x, data_x1, data_y)

if __name__ == '__main__':

    data_x, data_x1, data_y = load_data()

    n = data_x.shape[0]
    '''Normalizing test data b/w 0 and 1'''
    Xtest = data_x[:,2].reshape(-1,1)
    Xtest = preprocessData(Xtest[:,0])
    Xtest = Xtest.reshape(-1,1)
    Xtest = sortData(Xtest)

    Xtest1 = data_x1[:,2].reshape(-1,1)
    Xtest1 = preprocessData(Xtest1[:,0])
    Xtest1 = Xtest1.reshape(-1,1)
    Xtest1 = sortData(Xtest1)

    X = np.concatenate((Xtest, Xtest1), axis=1)

    ''''Normalizing training data b/w 0 and 1'''
    Xtrain = data_x[:,3].reshape(-1,1)
    Xtrain = preprocessData(Xtrain[:,0])
    Xtrain = Xtrain.reshape(-1,1)
    Xtrain = sortData(Xtrain)

    Xtrain1 = data_x1[:,3].reshape(-1,1)
    Xtrain1 = preprocessData(Xtrain1[:,0])
    Xtrain1 = Xtrain1.reshape(-1,1)
    Xtrain1 = sortData(Xtrain1)

    x = np.concatenate((Xtrain, Xtrain1), axis=1)

    ytrain = data_y[:,3].reshape(-1,1)
    ytrain = preprocessData(ytrain[:,0])
    ytrain = ytrain.reshape(-1,1)
    ytrain = sortData(ytrain)

    y = ytrain

    kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2))

    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)

    # Fit to data using Maximum Likelihood Estimation of the parameters
    gp.fit(X, y)

    # Make the prediction on the meshed x-axis (ask for MSE as well)
    y_pred, sigma = gp.predict(x, return_std=True)

    # Plot the function, the prediction and the 95% confidence interval based on
    # the MSE
    fig = plt.figure()
    plt.plot(x, y, 'bs', ms=2)#, label=u'$f(x) = x\,\sin(x)$')
    plt.plot(X, y)#, 'r.', markersize=1, label=u'Observations')
    plt.plot(x, y_pred, 'r--', label=u'Prediction', linewidth=1)
    plt.fill(np.concatenate([x, x[::-1]]),
             np.concatenate([y_pred - 1.9600 * sigma,
                            (y_pred + 1.9600 * sigma)[::-1]]),
             alpha=.1, color="#dddddd", ec='None', label='95% confidence interval')
    plt.xlabel('$x$')
    plt.ylabel('$f(x)$')
    plt.ylim(-10, 20)
    # plt.legend(loc='upper left')

    plt.show()
