#ifndef MARKERS_PAD_H
#define MARKERS_PAD_H
 
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin base
#include <leg_tracker/PersonArray.h>
#include <vector>

class QListView;
class QStandardItemModel;
class QModelIndex;
 
namespace rviz_markers_panel
{

class PersonPanel: public rviz::Panel
{
// statement Q_OBJECT 
Q_OBJECT
public:
    // constructor, QWidget will be used for initialize GUI
    PersonPanel( QWidget* parent = 0 );
    
    // Override for save configuration for rviz
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
 
  // public slots
public Q_SLOTS:
    // set topic for publisher
    void setTracker(const QModelIndex& index);

protected:
    void updatePerson(const leg_tracker::PersonArray::Ptr& persons);        
 
protected:
    // topic name 
    ros::NodeHandle       _nh;
    ros::Subscriber       _person_sub;
    ros::Publisher        _tracker_pub;

    QListView             *_person_list;
    QStandardItemModel    *_item_model;
    std::vector<int>      _person_id;  
    
};
 
} // end namespace rviz_plugin_tutorials
 
#endif