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
