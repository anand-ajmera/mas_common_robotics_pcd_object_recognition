#!/usr/bin/env python

import pandas as pd
import numpy as np
from scipy.optimize import minimize
from sklearn.cross_validation import StratifiedShuffleSplit
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import log_loss
import os
from os.path import join
import sys
import glob
import re

from mcr_object_recognition.trainer import ObjectClassifier
import roslib
import rospy

PACKAGE = 'mcr_object_recognition'

classifier_name = 'athome'
test_folder = "/home/anand/Studies/SEM2/Learning_Adaptivity/Project/At_home_training/testFeatures"

#Load classifiers
def load_classifier(classifier_name,clf_name,label_name):
    cfg_folder = join(roslib.packages.get_pkg_dir(PACKAGE), 'common', 'config')
    return ObjectClassifier.load(join(cfg_folder, classifier_name, clf_name),
                                 join(cfg_folder, classifier_name, label_name))

def load_test_file(test_folder):
    files = np.array(glob.glob(test_folder + '/*'))
    n = 0
    for f in files:
        features = np.ravel(np.genfromtxt(f,delimiter=','))
        if n < 1 :
            len_of_file = len(features)
            feature_pool = np.array(features)
            m = re.search('../testFeatures/(.+?)_',f)
            if m:
                true_labels = [m.group(1)]
        else:
            if(len(features) == len_of_file):
                feature_pool = np.vstack([feature_pool, features])
                m = re.search('../testFeatures/(.+?)_',f)
                if m:
                    true_labels.append(m.group(1))
        n += 1

    return [feature_pool,true_labels]




### building the classifiers
clfs = []

clf_svm = load_classifier(classifier_name,'classifier.pkl','label_encoder.pkl')
clf_dt = load_classifier(classifier_name,'classifier_dt.pkl','label_encoder_dt.pkl')
clf_rt = load_classifier(classifier_name,'classifier_rt.pkl','label_encoder_rt.pkl')

clfs.append(clf_svm)
clfs.append(clf_dt)
clfs.append(clf_rt)

[test_x,test_y] = load_test_file(test_folder) 

### finding the optimum weights

predictions = []
for clf in clfs:
	print('LogLoss {score}'.format(score=log_loss(test_y, clf.classifier.predict_proba(test_x))))
	predictions.append(clf.classifier.predict_proba(test_x))

def log_loss_func(weights):
    ''' scipy minimize will pass the weights as a numpy array '''
    final_prediction = 0
    for weight, prediction in zip(weights, predictions):
            final_prediction += weight*prediction

    return log_loss(test_y, final_prediction)
    
#the algorithms need a starting value, right not we chose 0.5 for all weights
#its better to choose many random starting points and run minimize a few times
starting_values = [0.5]*len(predictions)

#adding constraints  and a different solver as suggested by user 16universe
#https://kaggle2.blob.core.windows.net/forum-message-attachments/75655/2393/otto%20model%20weights.pdf?sv=2012-02-12&se=2015-05-03T21%3A22%3A17Z&sr=b&sp=r&sig=rkeA7EJC%2BiQ%2FJ%2BcMpcA4lYQLFh6ubNqs2XAkGtFsAv0%3D
cons = ({'type':'eq','fun':lambda w: 1-sum(w)})
#our weights are bound between 0 and 1
bounds = [(0,1)]*len(predictions)

res = minimize(log_loss_func, starting_values, method='SLSQP', bounds=bounds, constraints=cons)

print('Ensamble Score: {best_score}'.format(best_score=res['fun']))
print('Best Weights: {weights}'.format(weights=res['x']))