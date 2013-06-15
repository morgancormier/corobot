#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOption>

#include "ImageBig.h"

//This image is made to be a 640*480 image

ImageBig::ImageBig()
{


}


QRectF ImageBig::boundingRect() const
{
    qreal adjust = 2;
    return QRectF(0, 0,
                  640 + adjust, 480 + adjust);
}

void ImageBig::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{

    painter->drawImage(0,0,image);
}

void ImageBig::setImage(QImage& image)
//set the QImage
{
   this->image = image;

}

void ImageBig::saveImage()
{

    QString multifn;
    multifn ="../pictest.jpg";

    this->image.save(multifn,"JPG");

}


