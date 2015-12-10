#!/usr/bin/env python
import rospy
from appprs_main.srv import ClassifyLegs, ClassifyLegsResponse
#from std_msgs.msg import Float32MultiArray
from sklearn.svm import LinearSVC
import pickle

TRAINED_CLASSIFIER_PKL_STRING = """ccopy_reg\n_reconstructor\np0\n(csklearn.svm.classes\nLinearSVC\np1\nc__builtin__\nobject\np2\nNtp3\nRp4\n(dp5\nS\'loss\'\np6\nS\'l2\'\np7\nsS\'C\'\np8\nF1.0\nsS\'intercept_\'\np9\ncnumpy.core.multiarray\n_reconstruct\np10\n(cnumpy\nndarray\np11\n(I0\ntp12\nS\'b\'\np13\ntp14\nRp15\n(I1\n(I1\ntp16\ncnumpy\ndtype\np17\n(S\'f8\'\np18\nI0\nI1\ntp19\nRp20\n(I3\nS\'<\'\np21\nNNNI-1\nI-1\nI0\ntp22\nbI00\nS\'\\xda\\x8a\\x89x\\x84i\\xf2\\xbf\'\np23\ntp24\nbsS\'verbose\'\np25\nI0\nsS\'dual\'\np26\nI01\nsS\'fit_intercept\'\np27\nI01\nsS\'data_scaler\'\np28\ng0\n(csklearn.preprocessing.data\nStandardScaler\np29\ng2\nNtp30\nRp31\n(dp32\nS\'std_\'\np33\ng10\n(g11\n(I0\ntp34\ng13\ntp35\nRp36\n(I1\n(I13\ntp37\ng20\nI00\nS",d)\\xc3W\\xf97@@\\xe2z\\x98\\x91\\xe3\\xb5@\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\\x10\\x85\\xb4\\x13D\\xd5\\xd3@\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\\xedb)z\\x05\\x10\\x99@\\xe4\\xe8\\n\\x89Ql\\x10A\\xd2\\x88\\xf1\\xc3\\xd9\\xbe\\xe0@\'\\xf7\\x87\\x81\\xe3\\xe9\\x96@\\xc9Cj\\t\\xf0\\xe3+Ai\\xb0t\\x8a\\xe4\\xf6T@"\np38\ntp39\nbsS\'copy\'\np40\nI01\nsS\'with_mean\'\np41\nI01\nsS\'with_std\'\np42\nI01\nsS\'mean_\'\np43\ng10\n(g11\n(I0\ntp44\ng13\ntp45\nRp46\n(I1\n(I13\ntp47\ng20\nI00\nS\'\\xc5\\x9f\\xbew\\xce\\xd6G@o\\xad\\x10dx\\xb9\\xd0@\\x00\\x00\\x00\\x00\\x00j\\xe8@\\x00\\x00\\x00\\x00\\x00j\\xf8@\\x00\\x00\\x00\\x00\\x00j\\xe8@\\xd7;\\x9a\\xd0\\x86B\\xeb@\\x00\\x00\\x00\\x00\\x00j\\xf8@\\xf2F\\x95\\xd1ac\\xa2@\\x1d\\xef\\xcd\\x06\\x04V\\xed@0"\\xcd\\xceL\\xbd\\xf3@\\x94\\xb8\\xf7>\\xb7\\x86\\x9b@\\x07(\\xf0 PPBA\\xc4\\xffg\\xd6l\\xf5|@\'\np48\ntp49\nbsbsS\'class_weight_\'\np50\ng10\n(g11\n(I0\ntp51\ng13\ntp52\nRp53\n(I1\n(I2\ntp54\ng20\nI00\nS\'\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\'\np55\ntp56\nbsS\'penalty\'\np57\ng7\nsS\'multi_class\'\np58\nS\'ovr\'\np59\nsS\'random_state\'\np60\nNsS\'raw_coef_\'\np61\ng10\n(g11\n(I0\ntp62\ng13\ntp63\nRp64\n(I1\n(I1\nI14\ntp65\ng20\nI00\nS\'\\x16\\xb2\\xec8\\x9a\\xf8\\xd9\\xbf\\xdfe;\\xdc\\x15\\xb2\\xfc?\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x1dc\\xb1\\xde\\xfdw\\x00\\xc0\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\xbbz\\xc6\\x8f\\xb2\\xd1\\xf1?\\xa3\\xf8\\x92\\xbfT\\xaf\\xa2\\xbf\\xe8\\xa7A0\\x00\\x9e\\xf7\\xbf\\xdf\\x9a\\xce\\xe8\\xa0\\x80\\xc7?^\\x03\\xfbN\\xa8\\x08\\xd4?\\x90\\x9a\\xd7\\xb8\\xa5\\xa1\\x9f\\xbf\\xda\\x8a\\x89x\\x84i\\xf2\\xbf\'\np66\ntp67\nbsS\'_enc\'\np68\ng0\n(csklearn.preprocessing.label\nLabelEncoder\np69\ng2\nNtp70\nRp71\n(dp72\nS\'classes_\'\np73\ng10\n(g11\n(I0\ntp74\ng13\ntp75\nRp76\n(I1\n(I2\ntp77\ng20\nI00\nS\'\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\xf0?\'\np78\ntp79\nbsbsS\'tol\'\np80\nF0.0001\nsS\'coef_\'\np81\ng10\n(g11\n(I0\ntp82\ng13\ntp83\nRp84\n(I1\n(I1\nI13\ntp85\ng20\nI00\nS\'\\x16\\xb2\\xec8\\x9a\\xf8\\xd9\\xbf\\xdfe;\\xdc\\x15\\xb2\\xfc?\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x1dc\\xb1\\xde\\xfdw\\x00\\xc0\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\xbbz\\xc6\\x8f\\xb2\\xd1\\xf1?\\xa3\\xf8\\x92\\xbfT\\xaf\\xa2\\xbf\\xe8\\xa7A0\\x00\\x9e\\xf7\\xbf\\xdf\\x9a\\xce\\xe8\\xa0\\x80\\xc7?^\\x03\\xfbN\\xa8\\x08\\xd4?\\x90\\x9a\\xd7\\xb8\\xa5\\xa1\\x9f\\xbf\'\np86\ntp87\nbsS\'class_weight\'\np88\nNsS\'intercept_scaling\'\np89\nI1\nsb."""

def handle_classify_legs(req):
    features = req.features.data
    #print "Classifying features: %s" % [str(f) for f in features]
    try:
        scaled_features = classifier.data_scaler.transform(features)
        label = classifier.predict(scaled_features)[0]
        print "Label: %s" % label
        return ClassifyLegsResponse(int(label))
    except ValueError:
        return -1

def classify_legs_server():
    rospy.init_node('classify_legs_server')
    s = rospy.Service('classify_legs', ClassifyLegs, handle_classify_legs)
    print "Ready to classify legs"
    rospy.spin()

if __name__ == "__main__":
    classifier = pickle.loads(TRAINED_CLASSIFIER_PKL_STRING)
    classify_legs_server()
