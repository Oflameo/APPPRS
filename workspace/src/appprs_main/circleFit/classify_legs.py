#!/usr/bin/env python
import rospy
from appprs_main.srv import ClassifyLegs, ClassifyLegsResponse
#from std_msgs.msg import Float32MultiArray
from sklearn.svm import LinearSVC
import pickle
import numpy as np

def handle_classify_legs(req):
    features = np.array(req.features.data)
    features[features == -inf] = 0
    features[features == inf] = 0
    features[features == np.nan] = 0
    print "Classifying features: %s" % features
    scaled_features = classifier.data_scaler.transform(features)
    label = classifier.predict(scaled_features)[0]
    print "Label: %s" % label
    return ClassifyLegsResponse(label)

def classify_legs_server():
    rospy.init_node('classify_legs_server')
    s = rospy.Service('classify_legs', ClassifyLegs, handle_classify_legs)
    print "Ready to classify legs"
    rospy.spin()

if __name__ == "__main__":
    with open('trained_leg_svm_classifier.pkl', 'r') as f:
        classifier = pickle.load(f)
    classify_legs_server()
