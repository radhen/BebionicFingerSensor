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

from keras.layers.core import Dense, Dropout, Activation
from keras.optimizers import RMSprop
from keras.utils import np_utils


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
    d = {}
    df_1 = pd.DataFrame()
    train_x = np.zeros([15,151])
    train_y = np.zeros([15,])
    for maxForce in maxForces:
        df = pd.read_excel('baro_{}N.xlsx'.format(maxForce),header=None)
        # print (df[0])
        data=df.as_matrix() #converting into numpy array
        # print (data)
        for i in range(15):

            if data[0,i] == -20:
                train_y[i] = 0
            elif int(data[0,i]) == 0:
                train_y[i] = 1
            elif int(data[0,i]) == 20:
                train_y[i] = 2

            train_x[i] = (data[1:,i].tolist())
        # print (train_y)
        # print (train_y[10])
    return (train_x, train_y)

class RNN:
    '''
    RNN classifier
    '''
    def __init__(self, train_x, train_y, test_x, test_y, epoches=3, batch_size=1):

        self.epoches = epoches
        self.batch_size = batch_size

        self.train_x = train_x
        self.test_x = test_x
        self.train_y = np_utils.to_categorical(train_y, 3)
        self.test_y = np_utils.to_categorical(test_y, 3)

        self.model = Sequential()
        self.model.add(Dense(500, input_dim=151))
        self.model.add(Activation('relu'))
        self.model.add(Dropout(0.4))
        self.model.add(Dense(300))
        self.model.add(Activation('relu'))
        self.model.add(Dense(100))
        self.model.add(Activation('relu'))
        self.model.add(Dropout(0.4))
        self.model.add(Dense(3))
        self.model.add(Activation('softmax'))

        rms = RMSprop()
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
    test_x = np.zeros([1,151])
    train_x, train_y = load_data()
    test_x[0,:] = train_x[4]
    test_y = train_y[4]

    # train_x = train_x.T
    # test_x = test_x.T
    print (train_x.shape)
    print (test_x.shape)

    rnn = RNN(train_x, train_y, test_x, test_y)
    rnn.train()
    score, acc = rnn.evaluate()
    print('Test score:', score)
    print('Test accuracy:', acc)
