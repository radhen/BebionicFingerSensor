# BebionicFingerSensor
FingerSensor calibration data files for Bebionic hand

radhen patel <Radhen.Patel@colorado.edu>

Mon, Dec 24, 8:28 AM (5 days ago)

to Cooper.Simpson
In the top level of the GitHub repository radhen/Bebionics_FungerSensor

There is a /scripts folder. It has a /GP (gaussianprocesses) and /ML (machine learning) folder.

For /GP these two python files are relevant GP_sklearn.py and GPs.py. GP_sklearn.py is implemned uisng sklearn library. It has some good alrgorithms based on probabilistic and machine learning theory. Try to explore the API online.

For /ML, go inside classify_Probangle, that contains files for CNN and SVM to classify probing angle. Three angular locations (0, 20, -20 degree) from the paper I sent. Relevant pythons files are cnn_angle.py and svm_angle.py. Similarly in classify_SpatialLoc the relevant pythons files to classify five spatial locations (ref. paper) are cnn_spatial.py and svm_spatial.py.

I am using alignpeaks.m MATALAB files to preprocess the data for the above models. Recall the experimental setup I described to you last time we met. Excel files contains data for different peak loads (1N, 30N, 50N) appllied to the sensor. Python files are calling these Excel files and are further processesing the data for the models.  

Useful link for multiple output gaussianprocesses regression
https://stackoverflow.com/questions/50185399/multiple-output-gaussian-process-regression-in-scikit-learn
