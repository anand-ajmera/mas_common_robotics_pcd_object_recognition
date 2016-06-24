/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Anand Ajmera
 *
 */
#ifndef MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_ESF_H
#define MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_ESF_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <mcr_perception_msgs/PointCloud2List.h>
#include <sensor_msgs/PointCloud2.h> 
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/esf.h>

namespace mcr_object_recognition
{

class ObjectRecognitionESF : public nodelet::Nodelet
{
	
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloud;

    public:
        ObjectRecognitionESF(){}


    protected:
        ros::Publisher pub_feature_list_;

        ros::Subscriber sub_cluster_list_;

        ros::NodeHandle nh_;

        void onInit ();

        void clusterListCallback (const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds);
	
		void calculateFeatures(const PointCloud::ConstPtr &cloud, pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors);

};
PLUGINLIB_DECLARE_CLASS(mcr_object_recognition, ObjectRecognitionESF, mcr_object_recognition::ObjectRecognitionESF, nodelet::Nodelet);
}

#endif  // MCR_OBJECT_RECOGNITION_OBJECT_RECOGNITION_ESF_H
