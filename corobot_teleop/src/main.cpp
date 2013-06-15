#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QSplashScreen>
#include <unistd.h>


int main(int argc, char *argv[])
{


    QApplication a(argc, argv);
    Q_INIT_RESOURCE(images);

    QSplashScreen *splash = new QSplashScreen;

    splash->setPixmap(QPixmap("../resources/images/splash.jpg"));
    //sleep(1.5);
    splash->show();
    splash->showMessage(QObject::tr("ROS GUI is launching...."),Qt::AlignBottom,Qt::black);
    //ssleep(1);

    MainWindow w;

    w.show();

    splash->finish(&w);
    delete splash;

    a.setStyle("gtk+");
    a.exec();

    return 0;
}
