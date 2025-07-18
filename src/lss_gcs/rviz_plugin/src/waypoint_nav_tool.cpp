/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

/* Author: Dinesh Thakur - Modified for waypoint navigation */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <iostream>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/validate_floats.h>
#include <rviz/panel_dock_widget.h>

#include <geometry_msgs/PoseStamped.h>

#include "waypoint_nav_tool.h"

namespace waypoint_nav_plugin
{

WaypointNavTool::WaypointNavTool()
  : moving_flag_node_(NULL)
  , frame_dock_(NULL)
  , frame_(NULL)
  , server_("waypoint_nav", "", false)
  , unique_ind_(0)
{
  shortcut_key_ = 'l';
}

WaypointNavTool::~WaypointNavTool()
{
  M_StringToSNPtr::iterator sn_it;
  for(sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
      scene_manager_->destroySceneNode(sn_it->second);
  }

  delete frame_;
  delete frame_dock_;
}

void WaypointNavTool::onInitialize()
{
  flag_resource_ = "package://rviz_plugin/media/flag.dae";

  if(rviz::loadMeshFromResource(flag_resource_).isNull())
  {
    ROS_ERROR("WaypointNavTool: failed to load model resource '%s'.", flag_resource_.c_str());
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
  moving_flag_node_->attachObject(entity);
  moving_flag_node_->setVisible(false);

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  frame_ = new WaypointFrame(context_, &sn_map_, &server_, &unique_ind_, NULL, this);

  if (window_context)
    frame_dock_ = window_context->addPane("Waypoint Navigation", frame_);

  frame_->enable();

  //add Delete menu for interactive marker
  menu_handler_.insert("Delete", boost::bind(&WaypointNavTool::processFeedback, this, _1));
  menu_handler_.insert("Set Manual", boost::bind(&WaypointNavTool::processFeedback, this, _1));
}

// Activation and deactivation
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// activate() is called when the tool is started by the user, either
// by clicking on its button in the toolbar or by pressing its hotkey.
//
// First we set the moving flag node to be visible, then we create an
// rviz::VectorProperty to show the user the position of the flag.
// Unlike rviz::Display, rviz::Tool is not a subclass of
// rviz::Property, so when we want to add a tool property we need to
// get the parent container with getPropertyContainer() and add it to
// that.
//
// We wouldn't have to set current_flag_property_ to be read-only, but
// if it were writable the flag should really change position when the
// user edits the property.  This is a fine idea, and is possible, but
// is left as an exercise for the reader.
void WaypointNavTool::activate()
{
    if(missionIndex == 0){
//        std::cout<<"Masuk actieve 0\n";
        moving_flag_node_->setVisible(false);
    }else if(moving_flag_node_){
        moving_flag_node_->setVisible(true);
    }
}

// deactivate() is called when the tool is being turned off because
// another tool has been chosen.
//
// We make the moving flag invisible, then delete the current flag
// property.  Deleting a property also removes it from its parent
// property, so that doesn't need to be done in a separate step.  If
// we didn't delete it here, it would stay in the list of waypoints when
// we switch to another tool.
void WaypointNavTool::deactivate()
{
    if(missionIndex == 0){

    }else if(moving_flag_node_){
    moving_flag_node_->setVisible(false);
    }

}

// Handling mouse events

int WaypointNavTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if(!moving_flag_node_)
  {
    return Render;
  }

  double height = frame_->getDefaultHeight();
  Ogre::Vector3 intersection;
  Ogre::Quaternion quat;
  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height);
  if(rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection))
  {
    moving_flag_node_->setVisible(true);
    moving_flag_node_->setPosition(intersection);

    frame_->setWpLabel(intersection);

    //check if mouse pointer is near existing waypoint
    M_StringToSNPtr::iterator sn_it;
    for(sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
    {
      Ogre::Vector3 stored_pos = sn_it->second->getPosition();
      double distance = std::sqrt(pow(stored_pos.x - intersection.x,2) + pow(stored_pos.y - intersection.y,2));

      if(distance < 0.4)
      {
        moving_flag_node_->setVisible(false);

        //delete the waypoint if right clicked
        if(event.rightDown())
        {
          sn_it->second->detachAllObjects();
          std::stringstream wp_name;
          wp_name << "waypoint" << sn_it->first;
          std::string wp_name_str(wp_name.str());
          server_.erase(wp_name_str);
          server_.applyChanges();
          sn_map_.erase(sn_it);
          frame_->deleteSavadSpeed(sn_it->first);
          frame_->deleteActionWP(sn_it->first);
          frame_->deleteServoWP(sn_it->first);

          
          moving_flag_node_->setVisible(true);
          return Render | Finished;
        }
      }
    }

    //add a waypoint
    if(event.leftDown())
    {
        if(frame_->ui_->inser_mode_checkBox->isChecked())
            frame_->insertWpFromNavTool(intersection, quat);
        else
            makeIm(intersection, quat, frame_->missionSelected);
      return Render;
    } else if(event.rightDown())
    {
      //makeIm(intersection, quat);
      return Finished;
    }
  }
  else
  {
    moving_flag_node_->setVisible(false); // If the mouse is not pointing at the ground plane, don't show the flag.
  }
  return Render;
}

void WaypointNavTool::makeIm(const Ogre::Vector3& position, const Ogre::Quaternion& quat, int missionNow)
{
    unique_ind_++; //increment the index for unique marker names
    frame_->setSpeed(unique_ind_);
    frame_->setActionWP(unique_ind_);
    frame_->setServoWP(unique_ind_);
    std::stringstream wp_name, wp_desc;
    wp_name << "waypoint" << unique_ind_;
    double speed_wp = frame_->speed[missionIndex][unique_ind_-1];
    std::string sub_miss;

    if((missionIndex+1) % 2)
      sub_miss = "_in"; 
    else
      sub_miss = "_pre"; 

    wp_desc << "Waypoint " << unique_ind_ <<std::endl<<"Mission "<< (int)((missionIndex+1) / 2) << sub_miss
        << std::endl << "Speed " << speed_wp << std::endl << "Act " << frame_->action_wp[missionIndex][unique_ind_-1]
            << std::endl << "Servo " << frame_->servo_action_wp[missionIndex][unique_ind_-1];

    std::string wp_name_str(wp_name.str());
    std::string wp_desc_str(wp_desc.str());

    if(rviz::loadMeshFromResource(flag_resource_).isNull())
    {
      ROS_ERROR("WaypointNavTool: failed to load model resource '%s'.", flag_resource_.c_str());
      return;
    }

    // create a new flag in the Ogre scene and save it in a std::map.
    Ogre::SceneNode* sn_ptr = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
    sn_ptr->attachObject(entity);
    sn_ptr->setVisible(true);
    sn_ptr->setPosition(position);
    sn_ptr->setOrientation(quat);

    M_StringToSNPtr::iterator sn_entry = sn_map_.find(unique_ind_);

    if (sn_entry == sn_map_.end())
      sn_map_.insert(std::make_pair(unique_ind_, sn_ptr));
    else{
      ROS_WARN("%s already in map", wp_name_str.c_str());
      return;
    }

    int num_wp = sn_map_.size();

    geometry_msgs::PoseStamped pos;
    pos.pose.position.x = position.x;
    pos.pose.position.y = position.y;
    pos.pose.position.z = position.z;
    pos.pose.orientation.x = quat.x;
    pos.pose.orientation.y = quat.y;
    pos.pose.orientation.z = quat.z;
    pos.pose.orientation.w = quat.w;

    frame_->setWpCount(num_wp);

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.stamp = ros::Time::now();
    int_marker.header.frame_id = frame_->getFrameId().toStdString();

    int_marker.pose = pos.pose;
    int_marker.scale = 2;
    int_marker.name = wp_name_str;
    int_marker.description = wp_desc_str;

    // create a cylinder marker
    visualization_msgs::Marker cyn_marker;
    cyn_marker.type = visualization_msgs::Marker::CYLINDER;
    cyn_marker.scale.x = 2.0;
    cyn_marker.scale.y = 2.0;
    cyn_marker.scale.z = 0.2;

    float color[3];
        // if (missionNow == 1){
        //     color[0]= 1; color[1] = 0.2; color[2] = 0.2;
        // } else if (missionNow == 2){
        //     color[0]= 0.2; color[1] = 1; color[2] = 0.2;
        // } else{
        //     color[0]= 0.2; color[1] = 0.2; color[2] = 1;
        // }  
    
    if (missionNow % 2 ==0){
      color[0]= 1 *(missionNow%2); color[1] = 1*(missionNow%3); color[2] = 1;
    }
    else{
      color[0]= 1 *((missionNow-1)%2); color[1] = 1*((missionNow-1)%3); color[2] = 0.85;
    }



    cyn_marker.color.r = color[0];
    cyn_marker.color.g = color[1];
    cyn_marker.color.b = color[2];
    cyn_marker.color.a = 0.5;

    // create a non-interactive control which contains the marker
    visualization_msgs::InteractiveMarkerControl cyn_control;
    cyn_control.always_visible = true;
    cyn_control.markers.push_back(cyn_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(cyn_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 0.707106781;
    control.orientation.x = 0;
    control.orientation.y = 0.707106781;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control.name = "menu_delete";
    int_marker.controls.push_back(control);
//    std::cout << "PUSH BACK INT_MARKER ke " << int_marker.controls << " \n";

    server_.insert(int_marker);
    server_.setCallback(int_marker.name, boost::bind(&WaypointNavTool::processFeedback, this, _1));
    menu_handler_.apply(server_, int_marker.name);

    //Set the current marker as selected
    Ogre::Vector3 p = position;
    Ogre::Quaternion q = quat;
    frame_->setSelectedMarkerName(wp_name_str);
    frame_->setWpLabel(p);
    frame_->setPose(p, q);
//     frame_->setSpeed(unique_ind_);

    server_.applyChanges();
}

// void WaypointNavTool::deleteSelectedWP(std::string del_wp_name){
//     M_StringToSNPtr::iterator sn_entry =
//       sn_map_.find(std::stoi(del_wp_name.c_str()));
//     if (sn_entry == sn_map_.end())
//       ROS_ERROR("%s not found in map", del_wp_name.c_str());
//     else
//     {
//         //Delete selected waypoint
//         std::stringstream wp_name;
//         std::string wp_name_str(wp_name.str());
//         server_.erase(del_wp_name);

//         menu_handler_.reApply(server_);
//         server_.applyChanges();
//         sn_entry->second->detachAllObjects();
//         sn_map_.erase(sn_entry);

//         int num_wp = sn_map_.size();
//         frame_->setWpCount(num_wp);
//     }
// }

void WaypointNavTool::processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
    {

     M_StringToSNPtr::iterator sn_entry =
        sn_map_.find(std::stoi(feedback->marker_name.substr(8)));
      if (sn_entry == sn_map_.end())
        ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
      else
      {

        if(feedback->menu_entry_id == 1)
        {
          //Delete selected waypoint
          std::stringstream wp_name;
          wp_name << "waypoint" << sn_entry->first;
          frame_->deleteSavadSpeed(sn_entry->first);
          std::string wp_name_str(wp_name.str());
          server_.erase(wp_name_str);

          menu_handler_.reApply(server_);
          server_.applyChanges();
          sn_entry->second->detachAllObjects();
          sn_map_.erase(sn_entry);

		      int num_wp = sn_map_.size();
          frame_->setWpCount(num_wp);

        }
        else
        {
          //Set the pose manually from the line edits
          Ogre::Vector3 position;
          Ogre::Quaternion quat;

          frame_->getPose(position, quat);

          geometry_msgs::Pose pos;
          pos.position.x = position.x;
          pos.position.y = position.y;
          pos.position.z = position.z;

          pos.orientation.x = quat.x;
          pos.orientation.y = quat.y;
          pos.orientation.z = quat.z;
          pos.orientation.w = quat.w;

          sn_entry->second->setPosition(position);
          sn_entry->second->setOrientation(quat);

          frame_->setWpLabel(position);

          server_.setPose(feedback->marker_name, pos);
          server_.applyChanges();
        }
      }
    }
      break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
//        DEBUG("POSE UPDATE " << " YA");

        try{
          M_StringToSNPtr::iterator sn_entry = sn_map_.find(std::stoi(feedback->marker_name.substr(8)));

          if (sn_entry == sn_map_.end())
            ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
          else
          {
            geometry_msgs::PoseStamped pos;
            pos.pose = feedback->pose;

            Ogre::Vector3 position;
            position.x = pos.pose.position.x;
            position.y = pos.pose.position.y;
            position.z = pos.pose.position.z;

            sn_entry->second->setPosition(position);

            Ogre::Quaternion quat;
            quat.x = pos.pose.orientation.x;
            quat.y = pos.pose.orientation.y;
            quat.z = pos.pose.orientation.z;
            quat.w = pos.pose.orientation.w;

            sn_entry->second->setOrientation(quat);

            frame_->setWpLabel(position);
            frame_->setPose(position, quat);
            frame_->setSelectedMarkerName(feedback->marker_name);
            // frame_->setSpeed(unique_ind_);
          }
      }catch(int e){
            DEBUG("KESALAHAN UPDATE " << e);
        }
    }

     break;
  }
}

void WaypointNavTool::getMarkerPoses()
{
  M_StringToSNPtr::iterator sn_it;
  for (sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
    visualization_msgs::InteractiveMarker int_marker;

    std::stringstream wp_name;
    wp_name << "waypoint" << sn_it->first;
    std::string wp_name_str(wp_name.str());
    server_.get(wp_name_str, int_marker);

    ROS_ERROR("pos: %g %g %g", int_marker.pose.position.x,
    int_marker.pose.position.y,
    int_marker.pose.position.z);
  }
}

void WaypointNavTool::clearAllWaypoints()
{
  M_StringToSNPtr::iterator sn_it;
  for (sn_it = sn_map_.begin(); sn_it != sn_map_.end(); sn_it++)
  {
      scene_manager_->destroySceneNode(sn_it->second);
  }
  sn_map_.clear();
  unique_ind_ = 0;
}

// Loading and saving the waypoints
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
// Tools with a fixed set of Property objects representing adjustable
// parameters are typically just created in the tool's constructor and
// added to the Property container (getPropertyContainer()).  In that
// case, the Tool subclass does not need to override load() and save()
// because the default behavior is to read all the Properties in the
// container from the Config object.
//
// Here however, we have a list of named waypoint positions of unknown
// length, so we need to implement save() and load() ourselves.
//
// We first save the class ID to the config object so the
// rviz::ToolManager will know what to instantiate when the config
// file is read back in.
void WaypointNavTool::save(rviz::Config config) const
{
  config.mapSetValue("Class", getClassId());
  //rviz::Config waypoints_config = config.mapMakeChild("Waypoints");

  rviz::Config waypoints_config = config.mapMakeChild("WaypointsTool");

  waypoints_config.mapSetValue("topic", frame_->getOutputTopic());
  waypoints_config.mapSetValue("frame_id", frame_->getFrameId());
  waypoints_config.mapSetValue("default_height", frame_->getDefaultHeight());

  /*
  // The top level of this tool's Config is a map, but our waypoints
  // should go in a list, since they may or may not have unique keys.
  // Therefore we make a child of the map (``waypoints_config``) to store
  // the list.
  rviz::Config waypoints_config = config.mapMakeChild("Waypoints");

  // To read the positions and names of the waypoints, we loop over the
  // the children of our Property container:
  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for(int i = 0; i < num_children; i++)
  {
    rviz::Property* position_prop = container->childAt(i);
    // For each Property, we create a new Config object representing a
    // single waypoint and append it to the Config list.
    rviz::Config waypoint_config = waypoints_config.listAppendNew();
    // Into the waypoint's config we store its name:
    waypoint_config.mapSetValue("Name", position_prop->getName());
    // ... and its position.
    position_prop->save(waypoint_config);
  }
  */
}

// In a tool's load() function, we don't need to read its class
// because that has already been read and used to instantiate the
// object before this can have been called.
void WaypointNavTool::load(const rviz::Config& config)
{
  rviz::Config waypoints_config = config.mapGetChild("WaypointsTool");

  QString topic, frame;
  float height;
  if(!waypoints_config.mapGetString("topic", &topic))
    topic = "/waypoints";

  if(!waypoints_config.mapGetString("frame_id", &frame))
    frame = "map";

  waypoints_config.mapGetFloat("default_height", &height);

  frame_->setConfig(topic, frame, height);

/*
  // Here we get the "waypoints" sub-config from the tool config and loop over its entries:
  rviz::Config waypoints_config = config.mapGetChild("waypoints");
  int num_waypoints = waypoints_config.listLength();
  for(int i = 0; i < num_waypoints; i++)
  {
    rviz::Config waypoint_config = waypoints_config.listChildAt(i);
    // At this point each ``waypoint_config`` represents a single waypoint.
    //
    // Here we provide a default name in case the name is not in the config file for some reason:
    QString name = "waypoint " + QString::number(i + 1);
    // Then we use the convenience function mapGetString() to read the
    // name from ``waypoint_config`` if it is there.  (If no "Name" entry
    // were present it would return false, but we don't care about
    // that because we have already set a default.)
    waypoint_config.mapGetString("Name", &name);
    // Given the name we can create an rviz::VectorProperty to display the position:
    rviz::VectorProperty* prop = new rviz::VectorProperty(name);
    // Then we just tell the property to read its contents from the config, and we've read all the data.
    prop->load(waypoint_config);
    // We finish each waypoint by marking it read-only (as discussed
    // above), adding it to the property container, and finally making
    // an actual visible waypoint object in the 3D scene at the correct
    // position.
    prop->setReadOnly(true);
    getPropertyContainer()->addChild(prop);
    //makewaypoint(prop->getVector());
  }*/
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoint_nav_plugin::WaypointNavTool,rviz::Tool)

