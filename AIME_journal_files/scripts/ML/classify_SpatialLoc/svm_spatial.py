from sklearn import datasets
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import train_test_split

import pandas as pd
from pandas import Series
import numpy as np
from sklearn.preprocessing import StandardScaler
import math
from sklearn.svm import SVC
from scipy.stats import moment

from sklearn.model_selection import StratifiedKFold

from sklearn.decomposition import PCA
from matplotlib import pyplot as plt


def load_data():
    '''
    loads data
    '''
    maxForces = [1,5,30,50] # all maxForces
    X = np.zeros([200,151,2]) #(no. of examples , windowSize, channels(baro+ir))
    Y = np.zeros([200,])
    jj = 0
    for maxForce in maxForces:
        df_baro = pd.read_excel('{}N.xlsx'.format(maxForce),sheetname='baro',header=None)
        df_ir = pd.read_excel('{}N.xlsx'.format(maxForce),sheetname='ir',header=None)
        data_baro = df_baro.as_matrix() #converting into numpy array
        data_ir = df_ir.as_matrix()

        for i in range(50):
            # y-labels
            # if data_baro[0,i] == angles[0]:
            #     Y[i+jj] = 0
            # elif int(data_baro[0,i]) == angles[1]:
            #     Y[i+jj] = 1
            # elif int(data_baro[0,i]) == angles[2]:
            Y[i+jj] = data_baro[0,i]
            X[i+jj,:,0] = data_baro[1:,i].tolist()
            X[i+jj,:,1] = data_ir[1:,i].tolist()
        jj=jj+50
    # print
    return (X, Y)


def preprocessData(X):

    '''standarize data to zero mean and 1 std. dev.
    '''
    for i in range(200):
        # for j in range(2):
            series = Series(X[i,:])
            # prepare data for normalization
            values = series.values
            values = values.reshape((len(values), 1))
            # train the normalization
            scaler = StandardScaler()
            scaler = scaler.fit(values)
            # print('Mean: %f, StandardDeviation: %f' % (scaler.mean_, math.sqrt(scaler.var_)))
            # normalize the dataset and print
            standardized = scaler.transform(values)
            # print (standardized[:,0])
            X[i,:] = standardized[:,0]
    return (X)


def FeatureEng(X):
    '''Statistics moment calculation'''
    # X_new = np.zeros([X.shape[0],2])
    # for i in range(X.shape[0]):
    #     X_new[i,:]= moment(X[i,:,:],moment=5)

    '''Basic math operation for every pair of ir & baro reading'''
    X_new = np.zeros([X.shape[0],153])
    for i in range(X.shape[0]):
        X_new[i,:151]=  X[i,:,1] / X[i,:,0]
        # print (X_new[i,:,0].shape)
        np.append(X_new, max(X[i,:,1]))
        np.append(X_new, max(X[i,:,0]))

    return (X_new)


if __name__ == '__main__':

    X, Y = load_data()
    # print (max(X[0,:,0]))
    # print (Y[0:11])

    X_eng = FeatureEng(X) # input 3D array, output 2D

    X_eng = preprocessData(X_eng)

    # X_train, X_test, y_train, y_test = train_test_split(X_eng, Y, test_size=0.20, random_state=7)

    # '''contour map'''
    # # reduce features from 153 to 2 for plotting purpose
    # pca = PCA(n_components=2).fit(X_train)
    # pca_2d = pca.transform(X_train)
    #
    # pca1 = PCA(n_components=2).fit(X_test)
    # pca_2d1 = pca1.transform(X_test)
    #
    # # clf = SVC(kernel = 'poly', C = 1)
    # # clf.fit(pca_2d, y_train)
    # svm_model_linear = SVC(kernel = 'rbf', C = 2).fit(pca_2d, y_train)
    # svm_predictions = svm_model_linear.predict(pca_2d1)
    # acc = svm_model_linear.score(pca_2d1, y_test)
    # print (acc)
    #
    # for i in range(0, pca_2d.shape[0]):
    #     if y_train[i] == 0:
    #         c1 = plt.scatter(pca_2d[i,0],pca_2d[i,1],c='r',    s=50,marker='+')
    #     elif y_train[i] == 1:
    #         c2 = plt.scatter(pca_2d[i,0],pca_2d[i,1],c='g',    s=50,marker='o')
    #     elif y_train[i] == 2:
    #         c3 = plt.scatter(pca_2d[i,0],pca_2d[i,1],c='b',    s=50,marker='*')
    #     elif y_train[i] == 3:
    #         c3 = plt.scatter(pca_2d[i,0],pca_2d[i,1],c='b',    s=50,marker='^')
    #     elif y_train[i] == 4:
    #         c3 = plt.scatter(pca_2d[i,0],pca_2d[i,1],c='b',    s=50,marker='.')
    #
    # plt.legend([c1, c2, c3], ['O degree', '20 degree',   '-20 degree'])
    # x_min, x_max = pca_2d[:, 0].min() - 1,   pca_2d[:,0].max() + 1
    # y_min, y_max = pca_2d[:, 1].min() - 1,   pca_2d[:, 1].max() + 1
    # xx, yy = np.meshgrid(np.arange(x_min, x_max, .01),   np.arange(y_min, y_max, .01))
    # Z = svm_model_linear.predict(np.c_[xx.ravel(),  yy.ravel()])
    # Z = Z.reshape(xx.shape)
    # plt.contour(xx, yy, Z)
    # plt.title('Support Vector Machine Decision Surface')
    # plt.axis('off')
    #
    # plt.show()


    '''define 5-fold cross validation test harness'''
    kfold = StratifiedKFold(n_splits=6, shuffle=True, random_state=5)
    cvscores = []

    for train, test in kfold.split(X_eng, Y):
        svm_model_linear = SVC(kernel = 'rbf', C = 10).fit(X_eng[train], Y[train])
        svm_predictions = svm_model_linear.predict(X_eng[test])
        # model accuracy for X_test
        acc = svm_model_linear.score(X_eng[test], Y[test])
        print ('Accuracy:', acc)

        # creating a confusion matrix
        cm = confusion_matrix(Y[test], svm_predictions)
        print ('Confusion matrix:','\n',cm)

        cvscores.append(acc * 100)

    print("%.2f%% (+/- %.2f%%)" % (np.mean(cvscores), np.std(cvscores)))