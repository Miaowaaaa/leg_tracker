#include <stdio.h>
 
#include <QVBoxLayout>
#include <QListView>
#include <QStringList>
#include <QStandardItemModel>
#include <QModelIndex>
 
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <leg_tracker/Person.h>
#include <leg_tracker/PersonArray.h>
 
#include "rviz_markers_panel.h"
 
namespace rviz_markers_panel
{
// constructor
PersonPanel::PersonPanel( QWidget* parent )
  : rviz::Panel( parent )
{
    _person_sub = _nh.subscribe("/people_tracker",30,&PersonPanel::updatePerson,this);
    _tracker_pub = _nh.advertise<std_msgs::Int32>("/tracker_id",1);
    // create a input panel
    _person_list = new QListView;
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(_person_list);
    setLayout( layout );
    
    // connect signal with slots
    connect(_person_list, SIGNAL(clicked(QModelIndex)), this, SLOT(setTracker(QModelIndex)));
}
 
void PersonPanel::updatePerson(const leg_tracker::PersonArray::Ptr& persons)
{
    _item_model = new QStandardItemModel;
    QStringList str_list;
    int num_person = persons->people.size();
    _person_id.clear();
    for(int i = 0; i< num_person;i++)
    {
        _person_id.push_back(persons->people[i].id);
        str_list.append("Person ID: " + persons->people[i].id);
        QString str = static_cast<QString>(str_list.at(i));
        QStandardItem *item = new QStandardItem(str);
        _item_model->appendRow(item);
    }
    _person_list->setModel(_item_model);
    
    
}
 
// set Topic name
void PersonPanel::setTracker( const QModelIndex& index)
{
    // check the change of topic name.
    int row = index.row();
    std_msgs::Int32 msg;
    msg.data = row;
    _tracker_pub.publish(msg);
}   

 
// Override
void PersonPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}
 
// Override
void PersonPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}
 
}
 
// A rviz plugin statement
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_markers_panel::PersonPanel,rviz::Panel )