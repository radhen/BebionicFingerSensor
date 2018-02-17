## Some observation with current network

### DATE Feb 16 2018

### Network consists of two convolutional layers and one dense layer with 3 output neurons.

Keeping the batch_size=30 gives the max accuracy. This can be attributed to the way the training data (train_x) is set up. For every maxForce {1,5,30,50} there 30 examples (peaks, snapshots) of each baro and ir readings (len=151). The network however is NOT learning anything. The loss function stays the same. The testing and training accuracy is hence same 83.33%.

Why does the accuracy reduce so dramatically (83.33% to 12.5%) on changing the batch_size to + or - 2?

Adding one more dense layer of size 10 reduces the accuracy again to 12.5%. Why the same number?

Removing one of the convolutional layer reduces the accuracy to 12.5%. Adding it back brings the accuracy back to 83.33%.  

Magic number for filter >=13.
