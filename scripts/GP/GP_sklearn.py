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

from sklearn.metrics import r2_score

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
    df_newton = pd.read_excel('1^5^50N_gp_1.xlsx',sheetname='newton',header=None)
    df_baro = pd.read_excel('1^5^50N_gp_1.xlsx',sheetname='baro',header=None)
    df_ir = pd.read_excel('1^5^50N_gp_1.xlsx',sheetname='ir',header=None)

    data_baro = df_baro.values[:]
    data_ir = df_ir.values[:]
    data_y = df_newton.values[:]

    return (data_baro, data_ir, data_y)


def plot_2d(y_pred, sigma, X, x, y):
    '''Plot 2D function, the prediction and the 95% confidence interval based on
    the MSE'''
    fig = plt.figure()
    plt.plot(x, y, 'bs', ms=1.5, label='Measurement')
    plt.plot(X, y, markersize=1, label='Observations')
    plt.plot(X, y_pred, 'r--', label='Prediction', linewidth=1)
    plt.fill(np.concatenate([x, x[::-1]]),
             np.concatenate([y_pred - 1.96 * sigma,
                            (y_pred + 1.96 * sigma)[::-1]]),
             alpha=.5, color="#dddddd")

    plt.xlabel('Force(N)')
    plt.ylabel('IR')
    # plt.axis([-0.25, 1.25, -0.3, 1.2])
    plt.legend(loc='upper left')
    plt.show()

def plot_3d(y_pred, sigma, X, x, y):
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(x[:,0], x[:,1], y[:,0], 'bs', ms=2, label='Measurements')
    # ax.plot(X[:,0], X[:,1], y[:,0], label='')
    ax.plot(X[:,0], X[:,1], y_pred[:,0], 'r--', label='Estimate')
    ax.legend()
    ax.set_xlabel('Baro')
    ax.set_ylabel('IR')
    ax.set_zlabel('Force(N)')
    plt.show()


def gaussian_process(X,x,y):
    kernel = RBF(length_scale=0.01, length_scale_bounds=(10, 100))

    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)

    # Fit to data using Maximum Likelihood Estimation of the parameters
    gp.fit(X, y)

    # Make the prediction on the meshed x-axis (ask for MSE as well)
    y_pred, sigma = gp.predict(x, return_std=True)

    # MSE = metrics.mean_squared_error(y_pred, y)
    # print ('Mean Square Error:', MSE)
    RMSE = np.sqrt(((y_pred - y) ** 2).mean())
    print ('RMSE:', RMSE)
    print ('RMSE^2:', RMSE**2)

    return (y_pred, sigma, RMSE, RMSE**2)


def split_train_test(data_baro,data_ir,data_y):
    '''Using all the data, 50/50 train test split
    Train Test length need to be SAME for gp(X,y)'''
    # cross-validation
    cols = np.array([1,2,3,4,5,6,7,8,9,0])
    np.random.shuffle(cols)
    train_indices = cols[:5]
    test_indices = cols[-5:]

    X_tstBaro = []
    X_tstIR = []
    for i in test_indices:
        X_tstBaro.append(data_baro[:,i].tolist())
        X_tstIR.append(data_ir[:,i].tolist())
    X_tstBaro = np.array(X_tstBaro)
    X_tstIR = np.array(X_tstIR)
    X_tstBaro = X_tstBaro.reshape(-1,1)
    X_tstIR = X_tstIR.reshape(-1,1)
    X_tstBaro = preprocessData(X_tstBaro[:,0])
    X_tstIR = preprocessData(X_tstIR[:,0])
    X = np.concatenate((X_tstBaro, X_tstIR), axis=1)
    print (X.shape)

    X_trBaro = []
    X_trIR = []
    for i in train_indices:
        X_trBaro.append(data_baro[:,i].tolist())
        X_trIR.append(data_ir[:,i].tolist())
    X_trBaro = np.array(X_trBaro)
    X_trIR = np.array(X_trIR)
    X_trBaro = X_trBaro.reshape(-1,1)
    X_trIR = X_trIR.reshape(-1,1)
    X_trBaro = preprocessData(X_trBaro[:,0])
    X_trIR = preprocessData(X_trIR[:,0])
    x = np.concatenate((X_trBaro, X_trIR), axis=1)
    print (x.shape)

    y_tr = []
    for i in train_indices:
        y_tr.append(data_y[:,i].tolist())
    y_tr = np.array(y_tr)
    y_tr = y_tr.reshape(-1,1)
    y = preprocessData(y_tr[:,0])
    print (y.shape)

    return (X,x,y)

def split_train_test_SINGLE(data_baro,data_ir,data_y):
    '''one column test, one column train, from excel file'''
    X_tstBaro = data_baro[:,2].reshape(-1,1) # baro readings test
    X_tstBaro = preprocessData(X_tstBaro[:,0])
    # plt.plot (Xtest)
    # plt.show()

    X_tstIR = data_ir[:,2].reshape(-1,1) # ir readings test
    X_tstIR = preprocessData(X_tstIR[:,0])
    X = np.concatenate((X_tstBaro, X_tstIR), axis=1)
    # X = X_tstBaro

    X_trBaro = data_baro[:,3].reshape(-1,1) # baro readings train
    X_trBaro = preprocessData(X_trBaro[:,0])

    X_trIR = data_ir[:,3].reshape(-1,1) # ir readings train
    X_trIR = preprocessData(X_trIR[:,0])
    x = np.concatenate((X_trBaro, X_trIR), axis=1)
    # x = X_trBaro

    ytrain = data_y[:,3].reshape(-1,1)
    ytrain = preprocessData(ytrain[:,0])

    y = ytrain

    return (X,x,y)



if __name__ == '__main__':

    data_baro, data_ir, data_y = load_data()
    n = data_baro.shape[0]

    # randonly take m number of points
    m = 200
    indices = random.sample(range(n), m)

    # X,x,y = split_train_test(data_baro,data_ir,data_y)

    X,x,y = split_train_test_SINGLE(data_baro,data_ir,data_y)

    # if (randomSelection):
    #     X = Xtest[np.array(indices)] # select data points at randomly generated indices
    #     X = sortData(X) # since random selection sort again
    #     x = Xtrain[np.array(indices)] # select data points at randomly generated indices
    #     x = sortData(x)
    #     y = ytrain[np.array(indices)] # select data points at randomly generated indices
    #     y = sortData(y)

    # rmse_list = []
    # r2_list = []
    # for j in range(5):
    #     X,x,y = split_train_test(data_baro,data_ir,data_y)
    #     y_pred, rmse, r2 = gaussian_process(X,x,y)
    #
    # rmse_list.append(rmse)
    # r2_list.append(r2)
    #
    # print ('rmse mean', np.mean(rmse_list))
    # print ('rmse st dev', np.std(rmse_list))

    y_pred, sigma, rmse, r2 = gaussian_process(X,x,y)
    print ('r2 score', r2_score(y, y_pred))
    plot_2d(y_pred, sigma, X, x, y)
    # plot_3d(y_pred, sigma, X, x, y)
