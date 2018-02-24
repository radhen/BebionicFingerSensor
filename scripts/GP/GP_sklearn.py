import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.preprocessing import StandardScaler
import math
from sklearn import metrics

import matplotlib as mpl

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, RationalQuadratic, Matern

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

import random

randomSelection = False


def preprocessData(X):
    '''normalizing between 0 and 1 BAD IDEA'''
    X = (X - min(X))/float(max(X)-min(X))
    X = X.reshape(-1,1)
    X = sortData(X)
    return (X)

def sortData(A):
    l = A.tolist()
    l.sort()  # sorting helped in removing the sudden jumps.
    return (np.array(l))


def load_data():
    '''load data'''
    df_newton = pd.read_excel('301550N_gp.xlsx',sheetname='newton',header=None)
    df_baro = pd.read_excel('301550N_gp.xlsx',sheetname='baro',header=None)
    df_ir = pd.read_excel('301550N_gp.xlsx',sheetname='ir',header=None)

    data_x = df_baro.values[:]
    data_x1 = df_ir.values[:]
    data_y = df_newton.values[:]

    return (data_x, data_x1, data_y)


def plot_2d():
    '''Plot 2D function, the prediction and the 95% confidence interval based on
    the MSE'''
    fig = plt.figure()
    plt.plot(x, y, 'bs', ms=2)#, label=u'$f(x) = x\,\sin(x)$')
    plt.plot(X, y)#, 'r.', markersize=1, label=u'Observations')
    plt.plot(x, y_pred, 'r--', label=u'Prediction', linewidth=1)
    plt.fill(np.concatenate([x, x[::-1]]),
             np.concatenate([y_pred - 1.96 * sigma,
                            (y_pred + 1.96 * sigma)[::-1]]),
             alpha=.5, color="#dddddd")

    plt.xlabel('Baro & IR')
    plt.ylabel('f(baro,ir)$')
    pl.axis([-0.25, 1.25, -0.3, 1.2])
    plt.legend(loc='upper left')
    plt.show()

def plot_3d():
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x[:,0], x[:,1], y[:,0]*max(data_y[:,3]), 'bs', ms=2, label='Measurements')
    # ax.plot(X[:,0], X[:,1], y[:,0], label='')
    ax.plot(X[:,0], X[:,1], y_pred[:,0]*max(data_y[:,3]), 'r--', label='Estimate')
    ax.legend()
    ax.set_xlabel('Baro')
    ax.set_ylabel('IR')
    ax.set_zlabel('Force(N)')
    plt.show()



if __name__ == '__main__':

    data_x, data_x1, data_y = load_data()
    n = data_x.shape[0]

    # randonly take m number of points
    m = 200
    indices = random.sample(range(n), m)

    Xtest = data_x[:,2].reshape(-1,1) # baro readings test
    Xtest = preprocessData(Xtest[:,0])

    Xtest1 = data_x1[:,2].reshape(-1,1) # ir readings test
    Xtest1 = preprocessData(Xtest1[:,0])
    # print (Xtest.shape)
    X = np.concatenate((Xtest, Xtest1), axis=1)
    # X = Xtest

    Xtrain = data_x[:,3].reshape(-1,1) # baro readings train
    Xtrain = preprocessData(Xtrain[:,0])

    Xtrain1 = data_x1[:,3].reshape(-1,1) # ir readings train
    Xtrain1 = preprocessData(Xtrain1[:,0])

    x = np.concatenate((Xtrain, Xtrain1), axis=1)
    # x = Xtrain

    ytrain = data_y[:,3].reshape(-1,1)
    ytrain = preprocessData(ytrain[:,0])

    y = ytrain
    addNoise = False
    if (addNoise):
        '''adding noise'''
        dy = 0.05 + 0.01 * np.random.random(y.shape)
        noise = np.random.normal(0, dy)
        y += noise
        y = sortData(y)

    if (randomSelection):
        X = Xtest[np.array(indices)] # select data points at randomly generated indices
        X = sortData(X) # since random selection sort again
        x = Xtrain[np.array(indices)] # select data points at randomly generated indices
        x = sortData(x)
        y = ytrain[np.array(indices)] # select data points at randomly generated indices
        y = sortData(y)


    kernel = RBF(length_scale=0.01, length_scale_bounds=(10, 100))

    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)

    # Fit to data using Maximum Likelihood Estimation of the parameters
    gp.fit(X, y)

    # Make the prediction on the meshed x-axis (ask for MSE as well)
    y_pred, sigma = gp.predict(x, return_std=True)

    MSE = metrics.mean_squared_error(y_pred, y)
    print ('Mean Square Error:', MSE)

    print ('Mean accuray:', gp.score(X,y)) # Returns the mean accuracy on the given test data and labels.

    plot_3d()
