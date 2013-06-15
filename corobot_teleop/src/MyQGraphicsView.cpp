#include "MyQGraphicsView.h"

void MyQGraphicsView::wheelEvent ( QWheelEvent * event )
{
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;  // see QWheelEvent documentation
    _numScheduledScalings += numSteps;
    if (_numScheduledScalings * numSteps < 0)  // if user moved the wheel in another direction, we reset previously scheduled scalings
        _numScheduledScalings = numSteps;
 
    QTimeLine *anim = new QTimeLine(350, this);
    anim->setUpdateInterval(50);
 
    connect(anim, SIGNAL(valueChanged(qreal)), SLOT(scalingTime(qreal)));
    connect(anim, SIGNAL(finished()), SLOT(animFinished()));
    anim->start();
}

void MyQGraphicsView::scalingTime(qreal x)
{
    qreal factor = 1.0 + qreal(_numScheduledScalings) / 300.0;
    scale_factor *= factor;

  /*  if(scale_factor < 1.0)
	{
		scale_factor /= factor;
		if(_numScheduledScalings > 0)
			_numScheduledScalings--;
		else if(_numScheduledScalings <0)
			_numScheduledScalings++;
	}
	else
	{*/		
		scale(factor, factor);
	//}
}

void MyQGraphicsView::animFinished()
{
    if (_numScheduledScalings > 0)
        _numScheduledScalings--;
    else
        _numScheduledScalings++;
    sender()->~QObject();
}

MyQGraphicsView::MyQGraphicsView(QWidget* widget): QGraphicsView (widget)
{
	_numScheduledScalings = 0;
	scale_factor = 1.0;
}
