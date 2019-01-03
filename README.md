# BebionicFingerSensor
Contains python/MATLAB/C script and data files for the paper 
"Multi-Modal Fingertip Sensor with Proximity, Contact, and Force Localization Capabilities"
submitted at
AIME journal (link coming soon...)

In the top level of the GitHub repository radhen/Bebionics_FungerSensor

There is a /scripts folder. It has a /GP (gaussianprocesses) and /ML (machine learning) folder.

For /GP these two python files are relevant GP_sklearn.py and GPs.py. GP_sklearn.py is implemned uisng sklearn library. It has some good alrgorithms based on probabilistic and machine learning theory. Try to explore the API online.

For /ML, go inside classify_Probangle, that contains files for CNN and SVM to classify probing angle. Three angular locations (0, 20, -20 degree) from the paper I sent. Relevant pythons files are cnn_angle.py and svm_angle.py. Similarly in classify_SpatialLoc the relevant pythons files to classify five spatial locations (ref. paper) are cnn_spatial.py and svm_spatial.py.

I am using alignpeaks.m MATALAB files to preprocess the data for the above models. Recall the experimental setup I described to you last time we met. Excel files contains data for different peak loads (1N, 30N, 50N) appllied to the sensor. Python files are calling these Excel files and are further processesing the data for the models.  

Useful link for multiple output gaussianprocesses regression
https://stackoverflow.com/questions/50185399/multiple-output-gaussian-process-regression-in-scikit-learn

Useful link for neural network for multiple output regression
Google link: https://www.google.com/search?client=ubuntu&channel=fs&ei=h_4mXLmsMZTB7gK98IfwAw&q=multiple+output+regression+nueral+network+example&oq=multiple+output+regression+nueral+network+example&gs_l=psy-ab.3...1146847.1151085..1151762...1.0..0.135.1934.1j16......0....1..gws-wiz.......0i71j35i304i39j0i13i5i30j0i8i13i30j33i10.G1d2oa_WSow

1. https://stats.stackexchange.com/questions/261227/neural-network-for-multiple-output-regression
2. https://datascience.stackexchange.com/questions/16890/neural-network-for-multiple-output-regression
3. https://arxiv.org/pdf/1805.02716.pdf
