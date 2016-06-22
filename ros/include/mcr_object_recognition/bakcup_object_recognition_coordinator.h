/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#ifndef MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_COORDINATOR_H
#define MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_COORDINATOR_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mcr_perception_msgs/ObjectList.h>
#include <mcr_perception_msgs/RecognitionResultList.h>



namespace mcr_object_recognition
{

class ObjectRecognitionCoordinator : public nodelet::Nodelet
{
    public:
        ObjectRecognitionCoordinator();
        virtual ~ObjectRecognitionCoordinator();

    protected:
        virtual void onInit();
        void eventInCallback(const std_msgs::String::ConstPtr &msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr &image);
        void objectListCallback(const mcr_perception_msgs::ObjectList::ConstPtr &msg);
        void recognitionResultCallback(const mcr_perception_msgs::RecognitionResultList::ConstPtr &msg);
        void mergeResultsAndPublish();

    protected:
        ros::NodeHandle nh_;

        ros::Subscriber sub_event_in_;
        ros::Subscriber sub_object_list_;
        ros::Subscriber sub_recognition_result_;

        // event_in for object_detector
        ros::Publisher pub_event_in_;
        ros::Publisher pub_event_out_;
        ros::Publisher pub_object_list_;

        int number_of_recognition_nodes_;

        std::vector<mcr_perception_msgs::RecognitionResultList::ConstPtr> results_;
        mcr_perception_msgs::ObjectList object_list_;
};

PLUGINLIB_DECLARE_CLASS(mcr_object_recognition, ObjectRecognitionCoordinator, mcr_object_recognition::ObjectRecognitionCoordinator, nodelet::Nodelet);
}
#endif /* MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_COORDINATOR_H */
