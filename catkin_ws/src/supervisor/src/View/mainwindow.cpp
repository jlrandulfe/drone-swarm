#include "supervisor/View/mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>


MainWindow::MainWindow(QWidget *parent, Supervisor &sup) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // Init UI
    ui->setupUi(this);
    ui->shapeCombo->addItem(tr("V-shape"));
    ui->shapeCombo->addItem(tr("Grid"));
    ui->shapeCombo->addItem(tr("Polygon"));
    ui->droneAmountSpinBox->setRange(3,20);
    ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    qDebug() << droneAngle;

//    ui->plot->addGraph();
//    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssCircle);
//    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);

    // Connect
    connect( ui->shapeCombo, SIGNAL(currentIndexChanged(QString)), this, SLOT(on_shapeCombo_currentTextChanged(QString)));

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_droneAmountSpinBox_valueChanged(int arg1)
{
    droneAmount = arg1;
    qDebug() << droneAmount;
}

void MainWindow::on_shapeCombo_currentTextChanged(const QString &arg1)
{
    qDebug() << arg1;
    if (arg1.toStdString() == "Grid")
    {
        shapeSelection = "g";
        ui->droneAngleLabel->setEnabled(false);
        ui->droneAngleSpinbox->setEnabled(false);
        ui->droneAmountSpinBox->setRange(3,20);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    }
    if (arg1.toStdString() == "Polygon")
    {
        shapeSelection = "p";
        ui->droneAngleLabel->setEnabled(false);
        ui->droneAngleSpinbox->setEnabled(false);
        ui->droneAmountSpinBox->setRange(3,9);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-9)"));
    }
    if (arg1.toStdString() == "V-shape")
    {
        shapeSelection = "v";
        ui->droneAngleLabel->setEnabled(true);
        ui->droneAngleSpinbox->setEnabled(true);
        ui->droneAmountSpinBox->setRange(3,20);
        ui->droneAmountLabel->setText(tr("Amount of drones (3-20)"));
    }
    qDebug() << QString::fromStdString(shapeSelection);
}

void MainWindow::on_droneDistanceSpinbox_valueChanged(double arg1)
{
    droneDistance = static_cast<float>(arg1);
    qDebug() << droneDistance;
}

void MainWindow::on_droneAngleSpinbox_valueChanged(double arg1)
{
    droneAngle = static_cast<float>(arg1);
    qDebug() << droneAngle;
}


void MainWindow::on_applyButton_clicked()
{

}

void MainWindow::updateScene()
{
//    ui->droneScene->;
}

