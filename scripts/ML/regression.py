import pandas as pd
from pandas import concat
from sklearn.preprocessing import LabelEncoder
from math import sqrt
from numpy import concatenate
from matplotlib import pyplot
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM


def series_to_supervised(data, n_in=1, n_out=1, dropnan=True):
	"""
	Frame a time series as a supervised learning dataset.
	Arguments:
		data: Sequence of observations as a list or NumPy array.
		n_in: Number of lag observations as input (X).
		n_out: Number of observations as output (y).
		dropnan: Boolean whether or not to drop rows with NaN values.
	Returns:
		Pandas DataFrame of series framed for supervised learning.
	"""
	n_vars = 1 if type(data) is list else data.shape[1]
	df = DataFrame(data)
	cols, names = list(), list()
	## input sequence (t-n, ... t-1)
	for i in range(1, 0, -1):
		cols.append(df.shift(i))
		names += [('var%d(t-%d)' % (j+1, i)) for j in range(n_vars)]
	## forecast sequence (t, t+1, ... t+n)
	for i in range(0, 1):
		cols.append(df.shift(-i))
		if i == 0:
			names += [('var%d(t)' % (j+1)) for j in range(n_vars)]
		else:
			names += [('var%d(t+%d)' % (j+1, i)) for j in range(n_vars)]
	# put it all together
	agg = concat(cols, axis=1)
	agg.columns = names
	# drop rows with NaN values
	if dropnan:
		agg.dropna(inplace=True)
	return agg

if __name__ == '__main__':

    values = [x for x in range(10)]
    dataset = pd.read_excel('30N_forecast.xlsx')
    # print(dataset.values.shape[1])
    values = dataset.values
    ## normalize features
    scaler = MinMaxScaler(feature_range=(0, 1))
    scaled = scaler.fit_transform(values)
    ## frame as supervised learning
    reframed = series_to_supervised(scaled)
    # drop columns we don't want to predict
    # print (reframed)
    reframed.drop(reframed.columns[[2,3,4]], axis=1, inplace=True)


    ## split into train and test sets
    values = reframed.values
    n_train_examples = 604 #fist four peaks for this example
    train = values[:n_train_examples, :]
    test = values[n_train_examples:, :]
    ## split into input and outputs
    train_X, train_y = train[:, :-1], train[:, -1]
    test_X, test_y = test[:, :-1], test[:, -1]
    ## reshape input to be 3D [samples, timesteps, features]
    train_X = train_X.reshape((train_X.shape[0], 1, train_X.shape[1]))
    test_X = test_X.reshape((test_X.shape[0], 1, test_X.shape[1]))
    print(train_X.shape, train_y.shape, test_X.shape, test_y.shape)

    model = Sequential()
    model.add(LSTM(10, input_shape=(train_X.shape[1], train_X.shape[2])))
    model.add(Dense(1))
    model.compile(loss='mae', optimizer='adam')
    # fit network
    history = model.fit(train_X, train_y, epochs=5, batch_size=10, validation_data=(test_X, test_y), verbose=2, shuffle=False)
    # plot history
    # pyplot.plot(history.history['loss'], label='train')
    # pyplot.plot(history.history['val_loss'], label='test')
    # pyplot.legend()
    # pyplot.show()

    # make a prediction
    yhat = model.predict(test_X)
    test_X = test_X.reshape((test_X.shape[0], test_X.shape[2]))
    # print (yhat.shape)
    # print (test_X.shape)
    # # invert scaling for forecast
    inv_yhat = concatenate((yhat, test_X), axis=1)
    inv_yhat = scaler.inverse_transform(inv_yhat)
    # print (inv_yhat)
    inv_yhat = inv_yhat[:,2]
    # print (inv_yhat)
    # # # invert scaling for actual
    test_y = test_y.reshape((len(test_y), 1))
    inv_y = concatenate((test_y, test_X), axis=1)
    inv_y = scaler.inverse_transform(inv_y)
    print (inv_y)
    inv_y = inv_y[:,2]
    pyplot.plot(inv_y,'--r')
    # pyplot.plot(inv_yhat,'--b')
    pyplot.show()
    # # # calculate RMSE
    rmse = sqrt(mean_squared_error(inv_y, inv_yhat))
    print('Test RMSE: %.3f' % rmse)
