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

 #include <QGraphicsScene>
 #include <QPainter>
 #include <QStyleOption>
 #include <ctime>
 #include "Image.h"


/**
 * transform a Qimage of any size into a QGraphicsItem  
 */

 Image::Image()
 {
 }

 QRectF Image::boundingRect() const
 {
     qreal adjust = 2;
     int height = image.height();
     int width = image.width();
     return QRectF(0, 0,width + adjust, width + adjust);
 }

 void Image::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
 {
     painter->drawImage(0,0,image);
 }

 void Image::setImage(QImage& image)
 //set the QImage
 {
    this->image = image;

 }


void Image::saveImage(bool save) 
// save a picture of the current image stream on the hardrive. The date and time is given in the file name
{
    QString path;
    time_t t = time(0);   // get time now
    struct tm * now = localtime(&t);
    path = QString("../picture-") + QString::number(now->tm_year + 1900) + QString("-") + QString::number(now->tm_mon + 1) + QString("-") + QString::number(now->tm_mday) + QString(".jpg");

    this->image.save(path,"JPG");
}
