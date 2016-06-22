/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Anand Ajmera
 *
 */

#include <mcr_object_recognition/object_recognition_fpfh.h>


void mcr_object_recognition::ObjectRecognitionFPFH::onInit()
{
    nh_ = getPrivateNodeHandle();
    sub_cluster_list_ = nh_.subscribe("input",1,&mcr_object_recognition::ObjectRecognitionFPFH::clusterListCallback,this);
    pub_feature_list_ = nh_.advertise<mcr_perception_msgs::PointCloud2List>("output",1);

    sub_cluster_list_test_ = nh_.subscribe("input_test",1,&mcr_object_recognition::ObjectRecognitionFPFH::clusterListCallbackTest,this);
    pub_feature_list_test_ = nh_.advertise<sensor_msgs::PointCloud2>("output_test",1);

}

void mcr_object_recognition::ObjectRecognitionFPFH::clusterListCallbackTest (const sensor_msgs::PointCloud2::ConstPtr &cloudstest)
{
        // sensor_msgs::PointCloud2 features_list;

        pcl::PCLPointCloud2::Ptr pcl_input_cloud(new pcl::PCLPointCloud2);

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloudstest, *pcl_input_cloud);

        PointCloud::Ptr xyzcloud(new PointCloud);
        pcl::fromPCLPointCloud2(*pcl_input_cloud,*xyzcloud);
        NODELET_INFO_STREAM("cloud " << xyzcloud->points.size());
        // pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors(new pcl::PointCloud<pcl::ESFSignature640>());
        pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors(new pcl::PointCloud<pcl::ESFSignature640>());

        calculateFeatures(xyzcloud, descriptors);
        NODELET_INFO_STREAM("descriptors " << descriptors->points.size());

        sensor_msgs::PointCloud2 ros_pointcloud;
        pcl::PCLPointCloud2::Ptr pcl_contour(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*descriptors, *pcl_contour);
        pcl_conversions::fromPCL(*pcl_contour, ros_pointcloud);
        ros_pointcloud.header = cloudstest->header;
        NODELET_INFO_STREAM("ros width height " << ros_pointcloud.width << ros_pointcloud.height);

        pub_feature_list_test_.publish(ros_pointcloud);

}

void mcr_object_recognition::ObjectRecognitionFPFH::clusterListCallback (const mcr_perception_msgs::PointCloud2List::ConstPtr &clouds)
{
    mcr_perception_msgs::PointCloud2List features_list;

    for(int i = 0; clouds->pointclouds.size(); i++)
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

void mcr_object_recognition::ObjectRecognitionFPFH::calculateFeatures(const PointCloud::ConstPtr &cloud, pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors)
{

    PointCloud::Ptr keypoints(new PointCloud);

    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // Object for storing the FPFH descriptors for each point.
    //pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors(new pcl::PointCloud<pcl::ESFSignature640>());
    calculate_keypoints(cloud,keypoints);
    
    // Estimate the normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(keypoints);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);
 
    // // FPFH estimation object.
    // pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::ESFSignature640> fpfh;
    // fpfh.setSearchSurface(keypoints);
    // fpfh.setInputCloud(keypoints);
    // fpfh.setInputNormals(normals);
    // fpfh.setSearchMethod(kdtree);
    // // Search radius, to look for neighbors. Note: the value given here has to be
    // // larger than the radius used to estimate the normals.
    // fpfh.setRadiusSearch(0.05);
 
    // fpfh.compute(*descriptors);
    //cout<<"descriptors"<<*descriptors<<endl;

  
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud);

    esf.compute(*descriptors);

}

double mcr_object_recognition::ObjectRecognitionFPFH::computeCloudResolution(const PointCloud::ConstPtr& cloud)
{
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
 
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (! pcl_isfinite((*cloud)[i].x))
            continue;
 
        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
 
    return resolution;
}

void mcr_object_recognition::ObjectRecognitionFPFH::calculate_keypoints(const PointCloud::ConstPtr &cloud,PointCloud::Ptr &keypoints)
{
    int numberOfKeypoints = 10;
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setSalientRadius(6 * resolution);
    // Set the radius for the application of the non maxima supression algorithm.
    detector.setNonMaxRadius(4 * resolution);
    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    detector.setMinNeighbors(5);
    // Set the upper bound on the ratio between the second and the first eigenvalue.
    detector.setThreshold21(0.975);
    // Set the upper bound on the ratio between the third and the second eigenvalue.
    detector.setThreshold32(0.975);
    // Set the number of prpcessing threads to use. 0 sets it to automatic.
    detector.setNumberOfThreads(4);
 
    detector.compute(*keypoints);
    //cout<<"Keypoints"<<*keypoints<<endl;
    keypoints->width = numberOfKeypoints;
    keypoints->points.erase (keypoints->points.begin()+numberOfKeypoints,keypoints->points.end());

    
}
