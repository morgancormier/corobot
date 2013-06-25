#ifndef HOKUYO_H
#define HOKUYO_H


#include <QWidget>
#include <QtGui/QGraphicsView>
#include "Hokuyo_Points.h"
#include <QMessageBox>

// This widgets tries to show the laser range finder data or the infrared sensor values

class Hokuyo : public QWidget
{
  Q_OBJECT

  public:
    Hokuyo(QWidget *parent = 0);
    ~Hokuyo();
    Hokuyo_Points* hokuyo_points; // set of laser range finder points
    QMessageBox msgBox_hokuyo;
    bool showlines; // yes if the user want to show the lines
    double IR01_update; //latest first infrared sensor value
    double IR02_update; //latest second infrared sensor value
    bool showIR; // yes if the infrared sensors values has to be displayed
    bool showLRF; // yes if the laser range finder values has to be displayed

public slots:
    void hokuyo_update(Hokuyo_Points* hokuyo_points_);
    void IR_update(double IR01_new, double IR02_new);

  protected:
    virtual void paintEvent(QPaintEvent *event);

private:
    int flag;

};

#endif // HOKUYO_H
