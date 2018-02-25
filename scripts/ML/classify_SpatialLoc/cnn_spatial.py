from sklearn import datasets
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import train_test_split

import pandas as pd
from pandas import Series
import numpy as np
from sklearn.preprocessing import StandardScaler
import math
from sklearn.svm import SVC

from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import GRU
from keras.layers import Conv1D, Conv2D
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence
from keras.layers import MaxPooling1D, MaxPooling2D
from keras.preprocessing.text import Tokenizer
from keras.layers import Merge
from keras.layers.normalization import BatchNormalization

from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import RMSprop
from keras.utils import np_utils

from sklearn.model_selection import train_test_split
from sklearn.model_selection import StratifiedKFold


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

    '''standarize data to have 0 mean and 1 std. dev.
    '''
    for i in range(200):
        for j in range(2):
            series = Series(X[i,:,j])
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
            X[i,:,j] = standardized[:,0]
    return (X)


class NN:
    '''
    NN classifier
    '''
    def __init__(self, train_x, train_y, test_x, test_y, epoches=250, batch_size=5):

        self.epoches = epoches
        self.batch_size = batch_size

        self.train_x = train_x
        self.test_x = test_x

        # TODO: one hot encoding for train_y and test_y
        num_classes = 5
        self.train_y = np_utils.to_categorical(train_y, num_classes)
        self.test_y = np_utils.to_categorical(test_y, num_classes)
        # print (self.train_y.shape)
        # print (self.test_y)

        self.model = Sequential()
        self.model.add(Conv1D(2, kernel_size=10, strides=5, padding='valid',
                                    activation='relu',
                                    input_shape=(151,2)))
        # self.model.add(BatchNormalization())
        # self.model.add(MaxPooling1D(pool_size=2))
        self.model.add(Conv1D(2, kernel_size=3, strides=2, padding='valid',
                                    activation='relu',
                                    input_shape=(35,2)))
        # self.model.add(MaxPooling1D(pool_size=2))
        self.model.add(Flatten())
        self.model.add(Dense(5))
        self.model.add(Activation('softmax'))
        rms = RMSprop()
        self.model.compile(loss='categorical_crossentropy', optimizer=rms, metrics=['accuracy'])
        # print (self.model.summary())


    def train(self):
        '''
        fit in data and train model
        :return:
        '''
        # TODO: fit in data to train your model
        self.model.fit(self.train_x, self.train_y,
          batch_size=self.batch_size,
          epochs=self.epoches,
          validation_data=(self.test_x, self.test_y))
          #validation_split=0.2)

    def evaluate(self):
        '''
        evaluate trained model
        :return:
        '''
        return self.model.evaluate(self.test_x, self.test_y)


if __name__ == '__main__':

    X, Y = load_data()
    # print (X[:,:,0])
    # print (Y[0:11])

    X = preprocessData(X)

    # X_train, X_test, y_train, y_test = train_test_split(X, Y, test_size=0.20, random_state=22)

    # print (X_train.shape)
    # print (y_train.shape)

    '''define 5-fold cross validation test harness'''
    kfold = StratifiedKFold(n_splits=6, shuffle=True, random_state=27)
    cvscores = []
    for train, test in kfold.split(X, Y):
    #
        print ('\n','train X',X[train].shape,'test X',X[test].shape)
        print ('train Y',Y[train].shape,'test Y',Y[test])

        nn = NN(X[train], Y[train], X[test], Y[test])
        nn.train()
        score, acc = nn.evaluate()
        print('Test score:', score)
        print('Test accuracy:', acc)
        cvscores.append(acc * 100)
    print("%.2f%% (+/- %.2f%%)" % (np.mean(cvscores), np.std(cvscores)))
