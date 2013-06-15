#ifndef HOKUYO_H
#define HOKUYO_H


#include <QWidget>
#include <QtGui/QGraphicsView>
#include "Hokuyo_Points.h"
#include <QMessageBox>

class Hokuyo : public QWidget
{
  Q_OBJECT

  public:
    Hokuyo(QWidget *parent = 0);
    ~Hokuyo();
    Hokuyo_Points* hokuyo_points;
    QMessageBox msgBox_hokuyo;
    bool showlines;
    double IR01_update;
    double IR02_update;
    bool showIR;
    bool showLRF;

public slots:
    void hokuyo_update(Hokuyo_Points* hokuyo_points_);
    void IR_update(double IR01_new, double IR02_new);

  protected:
    virtual void paintEvent(QPaintEvent *event);

private:
            int flag;

};

#endif // HOKUYO_H
