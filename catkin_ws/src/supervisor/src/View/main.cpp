#include "supervisor/View/mainwindow.h"
#include <QApplication>
// #include "supervisor/Model/supervisor.hpp"

int main(int argc, char **argv)
{
    QApplication a(argc, argv);
   	ros::init(argc, argv, "srv_call");

  	ros::NodeHandle n;

	Supervisor sup(n);
	MainWindow w(0,sup);
    w.show();

    return a.exec();
}
