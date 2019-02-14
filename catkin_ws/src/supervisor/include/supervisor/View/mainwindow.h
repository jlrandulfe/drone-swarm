#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QDialog>
#include <QtGui>
#include <QtCore>
#include <QStyle>
#include <QDesktopWidget>
#include "supervisor/Model/supervisor.hpp"

#define STATIC 1
#define LINEAR 2
#define SINUSOIDAL 3

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent, Supervisor &sup);
    ~MainWindow();

private slots:

    void on_droneAmountSpinBox_valueChanged(int arg1);
    void on_shapeCombo_currentTextChanged(const QString &arg1);
    void on_droneDistanceSpinbox_valueChanged(double arg1);
    void on_droneAngleSpinbox_valueChanged(double arg1);
    void on_applyButton_clicked();
    void on_startButton_clicked();
    void on_stopButton_clicked();
    void on_initRangeSpinBox_valueChanged(double arg1);
    void on_simTimeSpinbox_valueChanged(double arg1);
    void on_simResSpinbox_valueChanged(int arg1);
    void on_testCombo_currentTextChanged(const QString &arg1);
    void on_freqSpinbox_valueChanged(double arg1);
    void on_xSpinbox_valueChanged(double arg1);
    void on_ySpinbox_valueChanged(double arg1);
    void on_noiseSpinbox_valueChanged(double arg1);
    void on_resetButton_clicked();


    void on_gainSpinbox_valueChanged(double arg1);

private:

    Ui::MainWindow *ui;
    Supervisor supervisor;
    void addPoint(double x, double y);
    void clearData();
    void plot();

    char shapeSelection;
    int droneAmount;
    float droneDistance;
    float droneAngle;
    float droneRandomRange;
    float simTime;
    float simRes;
    float frequency;
    float x;
    float y;
    int movementPattern;
    float noiseConstant;
    float controllerGain;
};

#endif // MAINWINDOW_H
