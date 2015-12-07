#-------------------------------------------------
#
# Project created by QtCreator 2015-12-06T06:23:27
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = KFTracker
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    laneDetection.cpp \
    CKalmanFilter.cpp

HEADERS += \
    laneDetection.h \
    CKalmanFilter.h
