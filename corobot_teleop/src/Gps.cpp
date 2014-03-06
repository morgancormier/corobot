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

#include "GPS.h"
#include <stdio.h>

//GPS Widget, the map is not displayed as of now as it would be against google maps rules without any payments to google. An open source solution has to be found

Gps::Gps(){
}


void Gps::set_map_type(QString value)
//changes the map type
{
}

void Gps::check_size()
//the size od the map if the size of the web widget
{
}

void Gps::set_zoom(int value)
//set the map zoom value
{
}

void Gps::update_coord(double lat, double lon)
//update the last coordinate value
{
}


void Gps::selection_changed()
//the itinerary selection has changed, therefore change the path that is displayed
{
}

void Gps::start_clicked()
//the button start has been clicked. So we start the itinary
{
    started = true;
    path.clear();
    QList<QListWidgetItem *> l = list_widget->findItems(QString("New Itinerary"),Qt::MatchExactly);
    if(l.count()==0)
        list_widget->insertItem(0,QString("New Itinerary"));
    list_widget->setCurrentItem(list_widget->item(0));

}

void Gps::stop_clicked()
//the button stop has been clicked. Stop the itinerary
{
    started = false;
    start_pos_set = false;

}

void Gps::save_clicked()
//the button save has been clicked. Save the itinerary in a file
{
    QList<QListWidgetItem *> l = list_widget->findItems(QString("New Itinerary"),Qt::MatchExactly);
    if(l.count() != 0){
        QString file = QFileDialog::getSaveFileName();
        FILE *f;
        QByteArray ba = file.toLatin1();
        if((f = fopen(ba.data(),"w+"))!=NULL){
            ba = path.toLatin1();
            fprintf(f,"%s",ba.data());
            fclose(f);
                QListWidgetItem *tmp = list_widget->takeItem(0);
                delete tmp;

            list_widget->addItem(file);
            loaded_files.append(path);
        }
    }
}

void Gps::setList(QListWidget *l)
//gives the pointer to the list widget to display the itineraries
{
    list_widget = l;
}

void Gps::load_clicked()
//the button stop has been clicked. Load the itinerary file given by the user
{
    QStringList files = QFileDialog::getOpenFileNames();

    FILE *f;
    QByteArray ba;
    foreach (QString  s, files) {
        ba = s.toLatin1();
        f = fopen(ba.data(),"r+");
        char* c = (char*) malloc (10000);

        if(fgets(c,10000,f)!=NULL){
            QString *s = new QString(c);
            loaded_files.append(*s);
            list_widget->addItem(ba.data());
	    list_widget->setCurrentItem(list_widget->item(list_widget->count() - 1));
            delete s;
        }
        fclose(f);


    }
    url.removeQueryItem("markers");
    check_size();
    selection_changed();

}

