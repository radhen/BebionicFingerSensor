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
from keras.layers import Conv1D
from keras.layers.embeddings import Embedding
from keras.preprocessing import sequence
from keras.layers import MaxPooling1D
from keras.preprocessing.text import Tokenizer
from keras.layers import Merge

from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.optimizers import RMSprop
from keras.utils import np_utils

from sklearn.model_selection import train_test_split


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
    angles = [0,20,-20] # all probing angles
    maxForces = [30] # all maxForces
    # df_1 = pd.DataFrame()
    train_x = np.zeros([30,151,2]) #(no. of examples , windowSize, channels(baro+ir))
    train_y = np.zeros([30,])
    for maxForce in maxForces:
        df_baro = pd.read_excel('baro_{}N.xlsx'.format(maxForce),header=None)
        df_ir = pd.read_excel('ir_{}N.xlsx'.format(maxForce),header=None)
        data_baro = df_baro.as_matrix() #converting into numpy array
        data_ir = df_ir.as_matrix()

        for i in range(30):
            # y-labels
            if data_baro[0,i] == -20:
                train_y[i] = 0
            elif int(data_baro[0,i]) == 0:
                train_y[i] = 1
            elif int(data_baro[0,i]) == 20:
                train_y[i] = 2
            # Xs (zipping baro and ir)
            # data = np.hstack(zip(data_baro[1:,i], data_ir[1:,i]))
            # data = np.concatenate((data_baro[1:,i],data_ir[1:,i]), axis=0)
            train_x[i,:,0] = (data_baro[1:,i].tolist())
            train_x[i,:,1] = (data_ir[1:,i].tolist())

    return (train_x, train_y)

class NN:
    '''
    NN classifier
    '''
    def __init__(self, train_x, train_y, test_x, test_y, epoches=15, batch_size=2):

        self.epoches = epoches
        self.batch_size = batch_size

        self.train_x = train_x
        self.test_x = test_x

        # TODO: one hot encoding for train_y and test_y
        num_classes = 3
        self.train_y = np_utils.to_categorical(train_y, num_classes)
        self.test_y = np_utils.to_categorical(test_y, num_classes)

        self.model = Sequential()
        self.model.add(Conv1D(15, kernel_size=(10), padding='valid',
                                    activation='relu',
                                    input_shape=(151,2)))
        self.model.add(MaxPooling1D(pool_size=4))
        # self.model.add(Flatten())
        self.model.add(Conv1D(15, kernel_size=(10), padding='same',
                                    activation='relu',
                                    input_shape=(151,2)))
        self.model.add(MaxPooling1D(pool_size=2))
        self.model.add(Flatten())
        self.model.add(Dense(10))
        self.model.add(Activation('relu'))
        self.model.add(Dense(5))
        self.model.add(Activation('relu'))
        # self.model.add(Dropout(0.4))
        self.model.add(Dense(3))
        self.model.add(Activation('softmax'))

        rms = RMSprop(lr=.01)
        self.model.compile(loss='categorical_crossentropy', optimizer=rms, metrics=['accuracy'])


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

    def evaluate(self):
        '''
        evaluate trained model
        :return:
        '''
        return self.model.evaluate(self.test_x, self.test_y)


if __name__ == '__main__':

    train_x, train_y = load_data()
    X_train, X_test, y_train, y_test = train_test_split(train_x, train_y, test_size=0.2, random_state=5)

    print ('X_train shape: '+str(X_train.shape))
    print (X_test.shape)
    print (y_train.shape)
    print (y_test)

    nn = NN(X_train, y_train, X_test, y_test)
    nn.train()
    score, acc = nn.evaluate()
    print('Test score:', score)
    print('Test accuracy:', acc)
