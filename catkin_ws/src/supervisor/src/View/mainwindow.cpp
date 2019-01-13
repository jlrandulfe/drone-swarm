#include "supervisor/View/mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>
#include <cstdlib>



MainWindow::MainWindow(QWidget *parent, Supervisor &sup) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    supervisor(sup)
{
    // Init UI
    ui->setupUi(this);
    ui->shapeCombo->addItem(tr("V-shape"));
    ui->shapeCombo->addItem(tr("Grid"));
    ui->shapeCombo->addItem(tr("Polygon"));
    ui->droneAmountSpinBox->setRange(3,20);
    ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    qDebug() << droneAngle;

    ui->applyButton->setStyleSheet("background-color: blue");
    ui->startButton->setStyleSheet("background-color: green");
    ui->stopButton->setStyleSheet("background-color: red");

//    ui->plot->addGraph();
//    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
//    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);

    // Connect
    connect( ui->shapeCombo, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_shapeCombo_currentTextChanged(QString)));

}

MainWindow::~MainWindow()
{
    delete ui;
    #include <cstdlib>
    system("rosnode kill execute");
    system("rosnode kill drome_swarm_sim");
    system("rosnode kill kalman_filter");
}

void MainWindow::on_shapeCombo_currentTextChanged(const QString &shape)
{
    if (shape.toStdString() == "Grid")
    {
        shapeSelection = 'g';
        ui->droneAngleLabel->setEnabled(false);
        ui->droneAngleSpinbox->setEnabled(false);
        ui->droneAmountSpinBox->setRange(3,20);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
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
    qDebug() << "Save settings button clicked";
    supervisor.setupSimulation(droneAmount, droneDistance, droneAngle, shapeSelection, droneRandomRange, simRes, simTime);
}

void MainWindow::on_startButton_clicked()
{
    qDebug() << "Start button clicked";
    supervisor.startSimulation();
}

void MainWindow::on_stopButton_clicked()
{
    qDebug() << "Stop button clicked";
    supervisor.stopSimulation();
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
