#-------------------------------------------------
#
# Project created by QtCreator 2018-12-07T15:25:22
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = View
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    simviewer.cpp \
    form.cpp

HEADERS  += mainwindow.h \
    simviewer.h \
    form.h

FORMS    += mainwindow.ui \
    form.ui
