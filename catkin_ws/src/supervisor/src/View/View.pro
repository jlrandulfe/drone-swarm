#-------------------------------------------------
#
# Project created by QtCreator 2018-12-07T15:25:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets gui #printsupport

TARGET = View
TEMPLATE = app
CONFIG += qt

SOURCES += main.cpp\
        mainwindow.cpp \
#        src/qcustomplot.cpp

HEADERS  += ../../include/supervisor/View/mainwindow.h \
#    include/supervisor/qcustomplot.h \

FORMS    += mainwindow.ui
