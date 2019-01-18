#include "../../include/supervisor/View/mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <cstdlib>
#include <csignal>



MainWindow::MainWindow(QWidget *parent, Supervisor &sup) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    supervisor(sup)
{
    shapeSelection = 'v';
    droneAmount = 3;
    droneDistance = 5.0;
    droneAngle = 15.0;
    droneRandomRange = 0.5;
    simTime = 400;
    simRes = 50;
    movementPattern = STATIC;
    frequency = 0.0;
    x = 0.1;
    y = 0.1;
    noiseConstant = 0.5;

    // Init UI
    ui->setupUi(this);
    ui->shapeCombo->addItem(tr("V-shape"));
    ui->shapeCombo->addItem(tr("Grid"));
    ui->shapeCombo->addItem(tr("Polygon"));

    ui->testCombo->addItem(tr("Static"));
    ui->testCombo->addItem(tr("Linear"));
    ui->testCombo->addItem(tr("Sinusoidal"));

    ui->amplitudeLabel->setEnabled(false);
    ui->velLabel->setEnabled(false);
    ui->slashLabel->setEnabled(true);
    ui->xLabel->setEnabled(false);
    ui->yLabel->setEnabled(false);
    ui->xSpinbox->setEnabled(false);
    ui->ySpinbox->setEnabled(false);
    ui->freqLabel->setEnabled(false);
    ui->freqSpinbox->setEnabled(false);

    ui->droneAmountSpinBox->setRange(3,20);
    ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    qDebug() << droneAngle;

    ui->applyButton->setStyleSheet("background-color: blue;color: rgb(0, 0, 0)");
    ui->startButton->setStyleSheet("background-color: rgb(0,255,30); color: rgb(0, 0, 0)");
    ui->stopButton->setStyleSheet("background-color: red; color: rgb(0, 0, 0)");
    ui->resetButton->setStyleSheet("background-color: rgb(255,220,0); color: rgb(0, 0, 0)");

    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(false);
    ui->resetButton->setEnabled(false);
    ui->startButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->stopButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->resetButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));

    ui->centralWidget->setGeometry(
        QStyle::alignedRect(
            Qt::LeftToRight,
            Qt::AlignRight,
            ui->centralWidget->size(),
            qApp->desktop()->availableGeometry()
        )
    );

    ui->splitter->setSizes(QList<int>() << 50 << 200);

//    ui->plot->addGraph();
//    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
//    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);

    // Connect
    connect( ui->shapeCombo, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_shapeCombo_currentTextChanged(QString)));

}

MainWindow::~MainWindow()
{
    system("killall -9 -e python");
    delete ui;
    system("rosnode kill execute");
    system("rosnode kill drone_swarm_sim");
    system("rosnode kill kalman_filter");
    system("rosnode kill View");
    system("rosnode kill controller");
    system("killall -9 -e roslaunch");
}

void MainWindow::on_shapeCombo_currentTextChanged(const QString &shape)
{
    if (shape.toStdString() == "Grid")
    {
        shapeSelection = 'g';
        ui->droneAngleLabel->setEnabled(false);
        ui->droneAngleSpinbox->setEnabled(false);
        ui->droneAmountSpinBox->setRange(4,20);
        ui->droneAmountLabel->setText(tr("Amount of drones (4-20)"));
    }
    else if (shape.toStdString() == "Polygon")
    {
        shapeSelection = 'p';
        ui->droneAngleLabel->setEnabled(false);
        ui->droneAngleSpinbox->setEnabled(false);
        ui->droneAmountSpinBox->setRange(3,9);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-9)"));
    }
    else if (shape.toStdString() == "V-shape")
    {
        shapeSelection = 'v';
        ui->droneAngleLabel->setEnabled(true);
        ui->droneAngleSpinbox->setEnabled(true);
        ui->droneAmountSpinBox->setRange(3,20);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    }
    std::cout << "Shape selected: " << shapeSelection << std::endl;
}

void MainWindow::on_droneDistanceSpinbox_valueChanged(double distance)
{
    droneDistance = static_cast<float>(distance);
    qDebug() << "Distance: " << droneDistance;
}

void MainWindow::on_droneAngleSpinbox_valueChanged(double angle)
{
    droneAngle = static_cast<float>(angle);
    qDebug() << "Angle: " << droneAngle;
}

void MainWindow::on_droneAmountSpinBox_valueChanged(int amount)
{
    droneAmount = amount;
    qDebug() << "Amount: " << droneAmount;
}

void MainWindow::on_applyButton_clicked()
{
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);
    ui->resetButton->setEnabled(false);

    ui->startButton->setStyleSheet("background-color: rgb(0,255,30); color: rgb(0, 0, 0)");
    ui->stopButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->resetButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));


    qDebug() << "\n\n---------------------";
    qDebug() << "Save settings button clicked";
    qDebug() << "Shape selected:\t" << shapeSelection;
    qDebug() << "Amount:\t\t" << droneAmount;
    qDebug() << "Angle:\t\t" << droneAngle;
    qDebug() << "Distance:\t" << droneDistance;
    qDebug() << "Init range:\t" << droneRandomRange;
    qDebug() << "Simulation time: " << simTime << " s";
    qDebug() << "Simulation timestep resolution:\t" << simRes << " ms";
    qDebug() << "Movement pattern:\t" << "(" << movementPattern << ")";
    qDebug() << "Frequency:\t" << frequency << "Hz";
    qDebug() << "x: " << x << ", y: " << y;
    qDebug() << "Noise constant:\t" << noiseConstant;
    qDebug() << "---------------------\n\n";

    supervisor.setupSimulation(
                droneAmount, droneDistance, droneAngle, shapeSelection, droneRandomRange,
                simRes, simTime, movementPattern, x, y, frequency
                );
}

void MainWindow::on_resetButton_clicked()
{
    qDebug() << "Reset button clicked";
    ui->resetButton->setEnabled(false);
    ui->resetButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));

    system("rosnode kill kalman_filter");
    system("rosnode kill controller");
    system("rosnode kill drone_swarm_sim");
    system("killall -9 -e python");
    system("(roslaunch supervisor reset.launch &)");

    ui->applyButton->setEnabled(true);
    ui->startButton->setEnabled(true);
    ui->stopButton->setEnabled(false);

    ui->applyButton->setStyleSheet("background-color: blue;color: rgb(0, 0, 0)");
    ui->startButton->setStyleSheet("background-color: rgb(0,255,30); color: rgb(0, 0, 0)");
    ui->stopButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
}

void MainWindow::on_startButton_clicked()
{
    qDebug() << "Start button clicked";
    ui->applyButton->setEnabled(false);
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(true);
    ui->resetButton->setEnabled(true);
    ui->applyButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->startButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->stopButton->setStyleSheet("background-color: red; color: rgb(0, 0, 0)");
    ui->resetButton->setStyleSheet("background-color: rgb(255,220,0); color: rgb(0, 0, 0)");


    supervisor.startSimulation();
}

void MainWindow::on_stopButton_clicked()
{
    qDebug() << "Stop button clicked";
    ui->applyButton->setEnabled(false);
    ui->startButton->setEnabled(false);
    ui->stopButton->setEnabled(false);
    ui->resetButton->setEnabled(true);
    ui->applyButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->startButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->stopButton->setStyleSheet(QString::fromUtf8("QPushButton:disabled; color: gray"));
    ui->resetButton->setStyleSheet("background-color: rgb(255,220,0); color: rgb(0, 0, 0)");

    supervisor.stopSimulation();
    system("rosnode kill kalman_filter");
}

void MainWindow::on_initRangeSpinBox_valueChanged(double range)
{
    droneRandomRange = range;
    qDebug() << "Init range: " << droneRandomRange;
}

void MainWindow::on_simTimeSpinbox_valueChanged(double simTime_)
{
    simTime = simTime_;
    qDebug() << "Simulation time: " << simTime << " s";
}

void MainWindow::on_simResSpinbox_valueChanged(int simRes_)
{
    simRes = simRes_;
    qDebug() << "Simulation timestep resolution: " << simRes << " ms";
}

void MainWindow::on_testCombo_currentTextChanged(const QString &pattern)
{
    if (pattern.toStdString() == "Static")
    {
        movementPattern = STATIC;
        ui->amplitudeLabel->setEnabled(false);
        ui->velLabel->setEnabled(false);
        ui->xLabel->setEnabled(false);
        ui->yLabel->setEnabled(false);
        ui->xSpinbox->setEnabled(false);
        ui->ySpinbox->setEnabled(false);
        ui->freqLabel->setEnabled(false);
        ui->freqSpinbox->setEnabled(false);
        ui->amplitudeLabel->setStyleSheet("font-weight: normal");
        ui->velLabel->setStyleSheet("font-weight: normal");
    }
    else if (pattern.toStdString() == "Linear")
    {
        movementPattern = LINEAR;
        ui->amplitudeLabel->setEnabled(false);
        ui->velLabel->setEnabled(true);
        ui->xLabel->setEnabled(true);
        ui->yLabel->setEnabled(true);
        ui->xSpinbox->setEnabled(true);
        ui->ySpinbox->setEnabled(true);
        ui->freqLabel->setEnabled(false);
        ui->freqSpinbox->setEnabled(false);
        ui->amplitudeLabel->setStyleSheet("font-weight: normal");
        ui->velLabel->setStyleSheet("font-weight: bold");
    }
    else if (pattern.toStdString() == "Sinusoidal")
    {
        movementPattern = SINUSOIDAL;
        ui->amplitudeLabel->setEnabled(true);
        ui->velLabel->setEnabled(false);
        ui->xLabel->setEnabled(true);
        ui->yLabel->setEnabled(true);
        ui->xSpinbox->setEnabled(true);
        ui->ySpinbox->setEnabled(true);
        ui->freqLabel->setEnabled(true);
        ui->freqSpinbox->setEnabled(true);
        ui->amplitudeLabel->setStyleSheet("font-weight: bold");
        ui->velLabel->setStyleSheet("font-weight: normal");
    }
    qDebug() << "Movement pattern: " << pattern << "(" << movementPattern << ")";
}

void MainWindow::on_freqSpinbox_valueChanged(double freq)
{
    frequency = freq;
    qDebug() << "Frequency: " << frequency << "Hz";
}

void MainWindow::on_xSpinbox_valueChanged(double xin)
{
    x = xin;
    qDebug() << "x: " << x << ", y: " << y;
}

void MainWindow::on_ySpinbox_valueChanged(double yin)
{
    y = yin;
    qDebug() << "x: " << x << ", y: " << y;
}

void MainWindow::on_noiseSpinbox_valueChanged(double noise)
{
    noiseConstant = noise;
    qDebug() << "Noise constant: " << noiseConstant;
}
