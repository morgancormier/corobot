#include "Hokuyo.h"
#include <QApplication>
#include <QPainter>

#include <QtGui>

#define MAX_RANGE 0.80 // 80cm

Hokuyo::Hokuyo(QWidget *parent)
    : QWidget(parent)
{
    showlines = false;
    flag = 1;
    hokuyo_points = NULL;
    //IR01_update = 0;
    //IR02_update = 0;
    hokuyo_points = new Hokuyo_Points [683];
    showIR = false;
    showLRF = false;

}

Hokuyo::~Hokuyo()

{
    //delete [] hokuyo_points;
    //hokuyo_points = NULL;
}


void Hokuyo::paintEvent(QPaintEvent *event)
{

//  QPen pen(Qt::black, 2, Qt::SolidLine);

//  QPainter painter(this);

//  painter.setPen(pen);
//  painter.drawLine(20, 40, 250, 40);

//  pen.setStyle(Qt::DashLine);
//  painter.setPen(pen);
//  painter.drawLine(20, 80, 250, 80);

//  pen.setStyle(Qt::DashDotLine);
//  painter.setPen(pen);
//  painter.drawLine(20, 120, 250, 120);

//  pen.setStyle(Qt::DotLishowlinesne);
//  painter.setPen(pen);
//  painter.drawLine(20, 160, 250, 160);

//  pen.setStyle(Qt::DashDotDotLine);
//  painter.setPen(pen);
//  painter.drawLine(20, 200, 250, 200);


//  QVector<qreal> dashes;
//  qreal space = 4;

//  dashes << 1 << space << 5 << space;

//  pen.setStyle(Qt::CustomDashLine);
//  pen.setDashPattern(dashes);
//  painter.setPen(pen);
//  painter.drawLine(20, 240, 250, 240);

    flag = 0;
	int center_h = (int) (this->parentWidget()->height() /2);
	int center_w = (int) (this->parentWidget()->width() /2);
    QPainter painter(this);

    //***************************************************************

        QPen pen2(Qt::black, 2, Qt::SolidLine);
        painter.setPen(pen2);
        painter.setBrush(Qt::red);
        QRectF rectangle(center_w - 2, center_h-2, 4, 4);
        painter.drawEllipse(rectangle);
    //*******************************************************

    if(showLRF)
    {
        QPen pen(Qt::black, 2, Qt::DashDotDotLine);

        painter.setPen(pen);

        painter.drawLine(20, 40, 120, 40);
    }

    else if(showIR)
    {
        QPen pen(Qt::black, 2, Qt::SolidLine);

        painter.setPen(pen);

        painter.drawLine(20, 40, 20 + center_h, 40);

    }
//**************************************************************
    if(showIR)
    {
        painter.setBrush(Qt::cyan);

        QPen pen1(Qt::black, 2, Qt::SolidLine);
       // painter.setBrush(brush);
	
	QPointF points_front[3];
	QPointF points_back[3];


	points_front[0] = QPointF(center_w,center_h);
	points_front[1] = QPointF(center_w - 50, center_h + (float)this->IR01_update * center_h);
	points_front[2] = QPointF(center_w + 50, center_h + (float)this->IR01_update * center_h);

    painter.setPen(pen1);
    painter.drawPolygon(points_front,3);

    painter.setBrush(Qt::darkCyan);

	points_back[0] = QPointF(center_w, center_h);
	points_back[1] = QPointF(center_w - 50, center_h - (float)this->IR02_update * center_h);
	points_back[2] = QPointF(center_w + 50, center_h - (float)this->IR02_update * center_h);


    //painter.setPen(pen1);
    painter.drawPolygon(points_back,3);
    }


    else if(showLRF)
    {
        QPen pen3(Qt::black, 2, Qt::DashDotDotLine);
        painter.setPen(pen3);

        int count = 0;

        if(hokuyo_points!=NULL)
        {
            for(int it=0;it<683;it++)
            {
                painter.drawPoint(hokuyo_points[it].x * 100 + 300, -(hokuyo_points[it].y * 100) + 300);
                if(showlines)
                painter.drawLine(300,300,hokuyo_points[it].x * 100 + 300, -(hokuyo_points[it].y * 100) + 300);
         //       qDebug("x value = %d  \n",hokuyo_points[it].x );
         //       qDebug("y value = %d  \n",hokuyo_points[it].y );
                count ++;
            }
        }

        //qDebug("How many points have been plotted ? %d \n",count);

        flag = 1;
   }


}

void Hokuyo::hokuyo_update(Hokuyo_Points* hokuyo_points_)
{

//    for(int it=0;it<683;it++)
//    {
//       qDebug("x value = %f  \n",hokuyo_points_[it].x );
//       qDebug("y value = %f  \n",hokuyo_points_[it].y );
//    }

    hokuyo_points = hokuyo_points_;

    //this->setAutoFillBackground(true);

    //this->repaint();

    this->update();

}

void Hokuyo::IR_update(double IR01_new, double IR02_new)
{
   this->IR01_update = IR01_new;
   this->IR02_update =  IR02_new;

  // qDebug("GOT IR vales IR01 = %f; IR02 = %f  \n",IR01_update,IR02_update); 

   //this->repaint();

   this->update();
}
