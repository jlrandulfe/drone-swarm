#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QDialog>
#include <QtGui>
#include <QtCore>
#include "supervisor/Model/supervisor.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /*explicit*/ MainWindow(QWidget *parent/* = 0*/, Supervisor &sup/* = NULL*/);
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

private:

    Ui::MainWindow *ui;
    Supervisor supervisor;
    void addPoint(double x, double y);
    void clearData();
    void plot();

    char shapeSelection;
    int droneAmount;
    float droneDistance;
    float droneAngle = 0.0f;
    float droneRandomRange;
    float simTime = 30;
    int simRes = 50;
};

#endif // MAINWINDOW_H
