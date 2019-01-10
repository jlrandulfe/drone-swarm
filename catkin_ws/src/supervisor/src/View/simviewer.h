#ifndef SIMVIEWER_H
#define SIMVIEWER_H

#include <QWidget>
#include "mainwindow.h"

class SimViewer : public QWidget
{
    Q_OBJECT
public:
    explicit SimViewer(QWidget *parent = 0);

signals:

public slots:
protected:
    void updateScene(QPaintEvent *);
private:
    QGraphicsScene *scene;
    QVector<QGraphicsEllipseItem *> ellipses;
    QGraphicsEllipseItem *ellipse;
};

#endif // SIMVIEWER_H
