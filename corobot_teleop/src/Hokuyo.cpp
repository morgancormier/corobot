/*
 * Copyright (c) 2009, CoroWare
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Hokuyo.h"
#include <QApplication>
#include <QPainter>

#include <QtGui>

#define MAX_RANGE 0.80 // 80cm

// This widgets tries to show the laser range finder data or the infrared sensor values

Hokuyo::Hokuyo(QWidget *parent)
    : QWidget(parent)
{
    showlines = false;
    flag = 1;
    hokuyo_points = NULL;
    hokuyo_points = new Hokuyo_Points [683];
    showIR = false;
    showLRF = false;

}

Hokuyo::~Hokuyo()
{
}


void Hokuyo::paintEvent(QPaintEvent *event)
// display the data on the winder
{

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

    if(showLRF) // if show laser range finder
    {
        QPen pen(Qt::black, 2, Qt::DashDotDotLine);

        painter.setPen(pen);

        painter.drawLine(20, 40, 120, 40);
    }

    else if(showIR) // if show small infrared sensor
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
                count ++;
            }
        }

        flag = 1;
   }


}

void Hokuyo::hokuyo_update(Hokuyo_Points* hokuyo_points_)
// update the laser range findre values
{
    hokuyo_points = hokuyo_points_;

    this->update();

}

void Hokuyo::IR_update(double IR01_new, double IR02_new)
// update the infrared sensor values
{
   this->IR01_update = IR01_new;
   this->IR02_update =  IR02_new;

   this->update();
}
