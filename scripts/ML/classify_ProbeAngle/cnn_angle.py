from scipy import signal
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import LabelEncoder
from scipy import signal

from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import GRU
from keras.layers import Conv1D, Conv2D
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence
from keras.layers import MaxPooling1D, MaxPooling2D
from keras.preprocessing.text import Tokenizer
# from keras.layers import Merge
from keras.layers.normalization import BatchNormalization

from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import RMSprop
from keras.utils import np_utils

from sklearn.model_selection import train_test_split
from sklearn.model_selection import StratifiedKFold

from pandas import Series
from sklearn.preprocessing import StandardScaler

import math


# fix random seed for reproducibility
seed = 7
np.random.seed(seed)

def make_patch_spines_invisible(ax):
    '''
    function used in PLOTTING
    '''
    ax.set_frame_on(True)
    ax.patch.set_visible(False)
    for sp in ax.spines.values():
        sp.set_visible(False)

def plot():
    '''
    PLOTTING stuff
    '''
    fig, ax1 = plt.subplots()
    plt.Figure(figsize=(10,5))
    ax1.plot(data[223:2978,2], 'b-',linewidth=0.5)
    ax1.set_ylabel('Force (16bit)', color='b')
    ax1.tick_params('y', colors='b')

    ax2 = ax1.twinx()
    ax2.plot(resample_baro, 'r-',linewidth=0.5)
    ax2.set_ylabel('Force(N)', color='r')
    ax2.tick_params('y', colors='r')

    ax3 = ax1.twinx()
    # Offset the right spine of par2.  The ticks and label have already been
    # placed on the right by twinx above.
    ax3.spines["right"].set_position(("axes", 1.2))
    # Having been created by twinx, par2 has its frame off, so the line of its
    # detached spine is invisible.  First, activate the frame but make the patch
    # and spines invisible.
    make_patch_spines_invisible(ax3)
    # Second, show the right spine.
    ax3.spines["right"].set_visible(True)
    ax3.plot(resample_ir, 'g-',linewidth=0.5)
    ax3.set_ylabel('IR (16bit)', color='g')
    ax3.tick_params(axis='y', colors='g')

    plt.title('{}degree {}N maxF'.format(angle, maxForce))
    fig.tight_layout()
    plt.show()

def load_data():
    '''
    loads data creates labels 0,1,2 for 0deg,20deg,-20deg
    '''
    angles = [0,20,-20] # all probing angles
    maxForces = [1,5,30,50] # all maxForces
    # df_1 = pd.DataFrame()
    X = np.zeros([120,151,2]) #(no. of examples , windowSize, channels(baro+ir))
    Y = np.zeros([120,])
    jj = 0
    for maxForce in maxForces:
        df_baro = pd.read_excel('{}N.xlsx'.format(maxForce),sheetname='Sheet1',header=None)
        df_ir = pd.read_excel('{}N.xlsx'.format(maxForce),sheetname='Sheet2',header=None)
        data_baro = df_baro.as_matrix() #converting into numpy array
        data_ir = df_ir.as_matrix()

        for i in range(30):
            # y-labels
            if data_baro[0,i] == angles[0]:
                Y[i+jj] = 0
            elif int(data_baro[0,i]) == angles[1]:
                Y[i+jj] = 1
            elif int(data_baro[0,i]) == angles[2]:
                Y[i+jj] = 2
            # Xs (zipping baro and ir)
            # data = np.hstack(zip(data_baro[1:,i], data_ir[1:,i]))
            # data = np.concatenate((data_baro[1:,i],data_ir[1:,i]), axis=0)
            X[i+jj,:,0] = data_baro[1:,i].tolist()
            X[i+jj,:,1] = data_ir[1:,i].tolist()
        jj=jj+30
    # print
    return (X, Y)

def preprocessData(X):

    '''normalizing data to zero mean and 1 std. dev.
    '''
    for i in range(120):
        # ----- normalizing between 0 and 1 ----- #
        # X[i,:,0] = (X[i,:,0] - min(X[i,:,0]))/float(max(X[i,:,0])-min(X[i,:,0]))
        # X[i,:,1] = (X[i,:,1] - min(X[i,:,1]))/float(max(X[i,:,1])-min(X[i,:,1]))

        # ----- normalizing with 0 mean and 1 std. dev. ----- #
        for j in range(2):
            series = Series(X[i,:,j])
            # prepare data for normalization
            values = series.values
            values = values.reshape((len(values), 1))
            # train the normalization
            scaler = StandardScaler()
            scaler = scaler.fit(values)
            print('Mean: %f, StandardDeviation: %f' % (scaler.mean_, math.sqrt(scaler.var_)))
            # normalize the dataset and print
            standardized = scaler.transform(values)
            # print (standardized[:,0])
            X[i,:,j] = standardized[:,0]
            # print(standardized.shape)
            # inverse transform and print
            # inversed = scaler.inverse_transform(standardized)
            # print(inversed.shape)
            # X[i,:,0] = inversed[:]
            # print (type(inversed))

    # print (X[0,:,0])
    # print (X[0,:,1])
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
        num_classes = 3
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
        # self.model.add(Conv1D(13, kernel_size=5, padding='valid',
        #                             activation='relu'))
        # self.model.add(Flatten())
        # self.model.add(LSTM(15, input_shape = (151,2)))
        # self.model.add(LSTM(5, input_shape = (151,2)))
        # self.model.add(LSTM(5, activation='tanh', recurrent_activation='hard_sigmoid'))
        # self.model.add(Flatten())
        # self.model.add(Dense(10))
        # self.model.add(Activation('relu'))
        self.model.add(Dense(3))
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

    X = preprocessData(X) # normalizing values

    '''' single split of data into training and testing '''
    # X[train], X[test], Y[train], Y[test] = train_test_split(X, Y, test_size=0.20, random_state=22)

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
    print (cvscores)

# 82.54% (+/- 4.58%)
# [85.71428656578064, 76.190477609634399, 85.71428656578064, 80.95238208770752, 88.88888955116272, 77.777779102325439]
