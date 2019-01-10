#include "simviewer.h"

SimViewer::SimViewer(QWidget *parent) : QWidget(parent)
{
}

void SimViewer::updateScene(QPaintEvent *)
{
    // Graphics scene
    scene = new QGraphicsScene(this);

    QBrush redBrush(Qt::red);
    QPen blackPen(Qt::black);
//    for (int i = 0; i < 3; i++)
//    {
//        ellipse = scene->addEllipse(i*10, i, i, i, blackPen, redBrush);
//        ellipses.append(ellipse);
//    }
    ellipse = scene->addEllipse(10, 10, 100, 100, blackPen, redBrush);
    QGraphicsView view(scene);
    view.show();
}

