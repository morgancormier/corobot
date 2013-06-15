#include "GPS.h"
#include <stdio.h>


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
//the button start has been clicked
{
    started = true;
    path.clear();
    QList<QListWidgetItem *> l = list_widget->findItems(QString("New Itinerary"),Qt::MatchExactly);
    if(l.count()==0)
        list_widget->insertItem(0,QString("New Itinerary"));
    list_widget->setCurrentItem(list_widget->item(0));

}

void Gps::stop_clicked()
//the button stop has been clicked
{
    started = false;
    start_pos_set = false;

}

void Gps::save_clicked()
//the button save has been clicked
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
//the button stop has been clicked
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

