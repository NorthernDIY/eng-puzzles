# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 20:09:15 2022

@author: NIcolas Hince DELL
"""
# Python version
import sys
print('Python: {}'.format(sys.version))
# scipy
import scipy
print('scipy: {}'.format(scipy.__version__))
# numpy
import numpy
print('numpy: {}'.format(numpy.__version__))
# matplotlib
import matplotlib
print('matplotlib: {}'.format(matplotlib.__version__))
# pandas
import pandas
print('pandas: {}'.format(pandas.__version__))
# scikit-learn
import sklearn
print('sklearn: {}'.format(sklearn.__version__))

# Load libraries
from pandas import read_csv
from pandas.plotting import scatter_matrix
from matplotlib import pyplot
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report
from sklearn.metrics import confusion_matrix
from sklearn.metrics import accuracy_score
from sklearn.linear_model import LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
from sklearn.naive_bayes import GaussianNB
from sklearn.svm import SVC

import numpy as np
import pandas as pd
import csv
import pickle

SimDataSorted=np.array(pickle.load(open("SimDataSorted", 'rb')))   

RealDataSorted=np.array(pickle.load(open("RealDataSorted", 'rb')))
l=0
for i in range(len(SimDataSorted[0])):
    for j in range(len(SimDataSorted[0][i])):
        if SimDataSorted[0][i][j]<(tuple(x+2 for x in RealDataSorted[0][i][j]))
        

# shape
print(SimDataSorted.shape)
print(RealDataSorted.shape)
# head
print(SimDataSorted.head(20))
print(RealDataSorted.head(20))

# descriptions
print(SimDataSorted.describe())
print(RealDataSorted.describe())

