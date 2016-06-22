/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Santosh Thoduka
 *
 */
#include <mcr_object_recognition/object_recognition_coordinator.h>
#include <mcr_perception_msgs/ObjectList.h>

using namespace mcr_object_recognition;

ObjectRecognitionCoordinator::ObjectRecognitionCoordinator()
{
}
ObjectRecognitionCoordinator::~ObjectRecognitionCoordinator()
{
}
void ObjectRecognitionCoordinator::onInit()
{
    nh_ = getPrivateNodeHandle();

    nh_.param<bool>("approximate_sync", approximate_sync_, false);
    nh_.param<int>("max_queue_size", max_queue_size_, 3);

    sub_results_filter_.subscribe(nh_, "results", max_queue_size_);
    sub_boxes_filter_.subscribe(nh_, "bounding_boxes", max_queue_size_);
    sub_poses_filter_.subscribe(nh_, "poses", max_queue_size_);

    pub_object_list_ = nh_.advertise<mcr_perception_msgs::ObjectList>("object_list", 1);

    label_visualizer_ = boost::make_shared<mcr::visualization::LabelVisualizer>(nh_, "labels", mcr::visualization::Color(mcr::visualization::Color::TEAL));

    if (approximate_sync_)
    {
        sync_input_a_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<mcr_perception_msgs::RecognitionResultList, mcr_perception_msgs::BoundingBoxList, geometry_msgs::PoseArray> > > (max_queue_size_);
        sync_input_a_->connectInput (sub_results_filter_, sub_boxes_filter_, sub_poses_filter_);
        sync_input_a_->registerCallback (bind (&mcr_object_recognition::ObjectRecognitionCoordinator::syncInputCallback, this, _1, _2, _3));
    }
    else
    {
        sync_input_e_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<mcr_perception_msgs::RecognitionResultList, mcr_perception_msgs::BoundingBoxList, geometry_msgs::PoseArray> > > (max_queue_size_);
        sync_input_e_->connectInput (sub_results_filter_, sub_boxes_filter_, sub_poses_filter_);
        sync_input_e_->registerCallback (bind (&mcr_object_recognition::ObjectRecognitionCoordinator::syncInputCallback, this, _1, _2, _3));
    }
}

void ObjectRecognitionCoordinator::syncInputCallback(const mcr_perception_msgs::RecognitionResultList::ConstPtr &results, const mcr_perception_msgs::BoundingBoxList::ConstPtr &boxes, const geometry_msgs::PoseArray::ConstPtr &poses)
{
    if (results->results.size() != poses->poses.size() || poses->poses.size() != boxes->bounding_boxes.size())
    {
        NODELET_ERROR_STREAM("different sizes for results, poses and bounding boxes" << results->results.size() << ", " << poses->poses.size() << ", " << boxes->bounding_boxes.size());
        return;
    }
    mcr_perception_msgs::ObjectList object_list;
    std::vector<std::string> labels;
    for (int i = 0; i < results->results.size(); i++)
    {
        mcr_perception_msgs::Object object;
        object.name = results->results[i].label;
        object.probability = results->results[i].probability;
        object.pose.pose = poses->poses[i];
        object.pose.header = poses->header;
        //optional?
//        object.bounding_box = boxes->bounding_box[i];
        object_list.objects.push_back(object);

        labels.push_back(object.name);
    }
    pub_object_list_.publish(object_list);
    label_visualizer_->publish(labels, *poses);
}
