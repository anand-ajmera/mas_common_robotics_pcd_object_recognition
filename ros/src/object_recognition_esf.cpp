/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Anand Ajmera
 *
 */

#include <mcr_object_recognition/object_recognition_esf.h>


void mcr_object_recognition::ObjectRecognitionESF::onInit()
{
    nh_ = getPrivateNodeHandle();
    sub_cluster_list_ = nh_.subscribe("input",1,&mcr_object_recognition::ObjectRecognitionESF::clusterListCallback,this);
    pub_feature_list_ = nh_.advertise<mcr_perception_msgs::PointCloud2List>("output",1);

}

void mcr_object_recognition::ObjectRecognitionESF::clusterListCallback (const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds)
{
    mcr_perception_msgs::PointCloud2List features_list;

    for(int i = 0; i < clouds->pointclouds.size(); i++)
    {
        pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

        // Convert to PCL data type
        pcl_conversions::toPCL(clouds->pointclouds[i], *pcl_input_cloud);

        PointCloud::Ptr xyzcloud(new PointCloud);
        pcl::fromPCLPointCloud2(*pcl_input_cloud,*xyzcloud);
        pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors(new pcl::PointCloud<pcl::ESFSignature640>());
        calculateFeatures(xyzcloud, descriptors);

        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*descriptors, *pcl_contour);
        pcl_conversions::fromPCL(*pcl_contour, ros_pointcloud);
        ros_pointcloud.header = clouds->header;
        features_list.pointclouds.push_back(ros_pointcloud);
    }

    pub_feature_list_.publish(features_list);

}

void mcr_object_recognition::ObjectRecognitionESF::calculateFeatures(const PointCloud::ConstPtr &cloud, pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors)
{
  
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud);

    esf.compute(*descriptors);

}

