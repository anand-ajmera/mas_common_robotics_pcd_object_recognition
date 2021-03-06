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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <mcr_perception_msgs/RecognitionResultList.h>
#include <mcr_perception_msgs/BoundingBoxList.h>
#include <geometry_msgs/PoseArray.h>
//#include <mcr_scene_segmentation/label_visualizer.h>



namespace mcr_object_recognition
{

namespace sync_policies = message_filters::sync_policies;

class ObjectRecognitionCoordinator : public nodelet::Nodelet
{
    public:
        ObjectRecognitionCoordinator();
        virtual ~ObjectRecognitionCoordinator();

    protected:
        virtual void onInit();
        void syncInputCallback(const mcr_perception_msgs::RecognitionResultList::ConstPtr &results, const mcr_perception_msgs::BoundingBoxList::ConstPtr &boxes, const geometry_msgs::PoseArray::ConstPtr &poses);

    protected:
        ros::NodeHandle nh_;

        message_filters::Subscriber<mcr_perception_msgs::RecognitionResultList> sub_results_filter_;
        message_filters::Subscriber<mcr_perception_msgs::BoundingBoxList> sub_boxes_filter_;
        message_filters::Subscriber<geometry_msgs::PoseArray> sub_poses_filter_;

        boost::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<mcr_perception_msgs::RecognitionResultList, mcr_perception_msgs::BoundingBoxList, geometry_msgs::PoseArray> > > sync_input_e_;
        boost::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<mcr_perception_msgs::RecognitionResultList, mcr_perception_msgs::BoundingBoxList, geometry_msgs::PoseArray> > > sync_input_a_;

        boost::shared_ptr<mcr::visualization::LabelVisualizer> label_visualizer_;

        ros::Publisher pub_object_list_;
        bool approximate_sync_;
        int max_queue_size_;
};

PLUGINLIB_DECLARE_CLASS(mcr_object_recognition, ObjectRecognitionCoordinator, mcr_object_recognition::ObjectRecognitionCoordinator, nodelet::Nodelet);
}
#endif /* MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_COORDINATOR_H */
