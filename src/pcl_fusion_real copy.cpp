#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/console/time.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <vector>
#include <ctime>
#include "tinyxml.h"
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#else
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#endif

// Sync message_filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <thermalvox_calib/PCAestimation.h>
#define DEBUG 1

using namespace std;
using namespace sensor_msgs;
using namespace Eigen;
using namespace message_filters;
using namespace pcl;
using namespace ros;

std::string cloud1_tp, cloud2_tp, cloud3_tp, cloud_fused_tp;
ros::Publisher cloud1_pub_, cloud2_pub_, cloud3_pub_, 
    cloud_fused_pub_, cloud_fused_ifiltered_pub_, cloud_fused_i_pub_, cloud_fused_zfiltered_pub_;
ros::Publisher cloud_paxes_pub;
std::string exParamFile_dir_ = "";

bool use_vox_filter_ = false;
double voxel_grid_size_ = 0.01;
double range_1 = 0.0, range_2 = 0.0, range_3 = 0.0;
Eigen::Matrix4d Tr;


pcl::PointCloud<pcl::PointXYZI>::Ptr range_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& pc, double inlier_r_min, double inlier_r_max)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for(auto i = pc->begin(); i < pc->end(); i++)
    {
        pcl::PointXYZI p;
        p.x = (*i).x;
        p.y = (*i).y;
        p.z = (*i).z;
        p.intensity = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
        temp_cloud->points.push_back(p);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_range;
    pass_range.setFilterFieldName("intensity");
    pass_range.setFilterLimits(inlier_r_min, inlier_r_max);
    pass_range.setInputCloud(temp_cloud);
    pass_range.setNegative (false);    // inliers
    pass_range.filter(*pc_);

    return pc_;
}

Eigen::Matrix4d load_exParam(const char* filename)
{
    ifstream loadfile;
    loadfile.open(filename);
    ROS_INFO("<<<<<<<<<<< LOADING FILE %s", filename);
    if(!loadfile)
    {
        ROS_WARN("Opening file faild!");
        // return false;
    }
    int i = 0;
    std::string line;
    Eigen::Matrix4d Tr;

    while (getline(loadfile, line))
    {
        // cout << line << endl;

        std::istringstream sin(line);
        vector<string> fields;
        std::string field;
        while(getline(sin,field,','))
        {
            fields.push_back(field);
        }

        for(int j = 0; j < 4; j++)
        {
            Tr(i,j) = atof(fields[j].c_str());
        }
        i++;
    }

    // return true;
    return Tr;
}


//Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& msg_pc1, const sensor_msgs::PointCloud2ConstPtr& msg_pc2)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor1_cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                        sensor2_cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                        sensor3_cloud (new pcl::PointCloud<pcl::PointXYZI>),
                                        fused_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    fromROSMsg(*msg_pc1, *sensor1_cloud);
    fromROSMsg(*msg_pc2, *sensor2_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud1 (new pcl::PointCloud<pcl::PointXYZI>),
                                        temp_cloud2 (new pcl::PointCloud<pcl::PointXYZI>);
    temp_cloud1 = range_filter(sensor1_cloud, 0.0, range_1);
    pcl::copyPointCloud(*temp_cloud1, *sensor1_cloud);
    temp_cloud2 = range_filter(sensor2_cloud, 0.0, range_2);
    pcl::copyPointCloud(*temp_cloud2, *sensor2_cloud);

    sensor_msgs::PointCloud2 cloud1_ros;
    pcl::toROSMsg(*sensor1_cloud, cloud1_ros);
    cloud1_ros.header = msg_pc1->header;
    cloud1_pub_.publish(cloud1_ros);
    sensor_msgs::PointCloud2 cloud2_ros;
    pcl::toROSMsg(*sensor2_cloud, cloud2_ros);
    cloud2_ros.header = msg_pc1->header;
    cloud2_pub_.publish(cloud2_ros);


    ROS_WARN("<<<<<<<<<<<<<<< Point Cloud Fusion <<<<<<<<<<<<<<<");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1_world (new pcl::PointCloud<pcl::PointXYZI>),
                                        cloud2_world (new pcl::PointCloud<pcl::PointXYZI>),
                                        cloud3_world (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*sensor1_cloud, *cloud1_world, Tr);
    pcl::copyPointCloud(*sensor2_cloud, *cloud2_world);
    *fused_cloud += *cloud1_world;
    // cout << "1st fused_cloud.size = " << fused_cloud->points.size() << endl;
    *fused_cloud += *cloud2_world;
    // cout << "2nd fused_cloud.size = " << fused_cloud->points.size() << endl;
    // *fused_cloud += *cloud3_world;
    // cout << "3rd fused_cloud.size = " << fused_cloud->points.size() << endl;

    // ********************** 2. Downsampling *********************
    if(use_vox_filter_)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(fused_cloud);
        sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
        sor.filter(*temp_cloud);

        cout << "PointCloud size after filter: " << temp_cloud->points.size() << endl;
        pcl::copyPointCloud(*temp_cloud, *fused_cloud);
    }
   
    sensor_msgs::PointCloud2 fused_cloud_ros;
    pcl::toROSMsg(*fused_cloud, fused_cloud_ros);
    fused_cloud_ros.header = msg_pc1->header;
    cloud_fused_pub_.publish(fused_cloud_ros);

   


    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    for(auto i = fused_cloud->begin(); i < fused_cloud->end(); i++)
    {
        pcl::PointXYZI p;
        p.x = (*i).x;
        p.y = (*i).y;
        p.z = (*i).z;
        p.intensity = sqrt(pow(p.x,2) + pow(p.y,2) + pow(p.z,2));
        temp_cloud->points.push_back(p);
    }
    // cout << "temp_cloud->points.size = " << temp_cloud->points.size() << endl;
    sensor_msgs::PointCloud2 fused_cloud_i_ros;
    pcl::toROSMsg(*temp_cloud, fused_cloud_i_ros);
    fused_cloud_i_ros.header = msg_pc1->header;
    cloud_fused_i_pub_.publish(fused_cloud_i_ros);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr fused_cloud_zfiltered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0f, 2.0f);
    pass_z.setInputCloud(temp_cloud);
    pass_z.setNegative (false);    // inliers
    pass_z.filter(*fused_cloud_zfiltered);

    sensor_msgs::PointCloud2 fused_cloud_zfiltered_ros;
    pcl::toROSMsg(*fused_cloud_zfiltered, fused_cloud_zfiltered_ros);
    fused_cloud_zfiltered_ros.header = msg_pc1->header;
    cloud_fused_zfiltered_pub_.publish(fused_cloud_zfiltered_ros);
    

    pcl::PointCloud<pcl::PointXYZI>::Ptr fused_cloud_ifiltered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass_i;
    pass_i.setFilterFieldName("intensity");
    pass_i.setFilterLimits(0.0f, 4.5f);
    pass_i.setInputCloud(fused_cloud_zfiltered);
    pass_i.setNegative (false);    // inliers
    pass_i.filter(*fused_cloud_ifiltered);

    sensor_msgs::PointCloud2 fused_cloud_ifiltered_ros;
    pcl::toROSMsg(*fused_cloud_ifiltered, fused_cloud_ifiltered_ros);
    fused_cloud_ifiltered_ros.header = msg_pc1->header;
    cloud_fused_ifiltered_pub_.publish(fused_cloud_ifiltered_ros);
    

    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_fusion_real");
    ros::NodeHandle nh_;

    if(ros::param::get("~cloud1_tp", cloud1_tp)
    && ros::param::get("~cloud2_tp", cloud2_tp)
    && ros::param::get("~cloud_fused_tp", cloud_fused_tp)
    && ros::param::get("~use_vox_filter", use_vox_filter_)
    && ros::param::get("~voxel_grid_size", voxel_grid_size_))
    {
        ROS_INFO("Retrived param 'cloud1_tp': %s", cloud1_tp.c_str());
        ROS_INFO("Retrived param 'cloud2_tp': %s", cloud2_tp.c_str());
        ROS_INFO("Retrived param 'cloud_fused_tp': %s", cloud_fused_tp.c_str());
        ROS_INFO("Retrived param 'use_vox_filter': %d", use_vox_filter_);
        ROS_INFO("Retrived param 'voxel_grid_size': %f", voxel_grid_size_);
        
    }

    if(ros::param::get("~range_1", range_1)
    && ros::param::get("~range_2", range_2)
    && ros::param::get("~range_3", range_3))
    {
        ROS_INFO("Retrived param 'range_1': %f", range_1);
        ROS_INFO("Retrived param 'range_2': %f", range_2);
        ROS_INFO("Retrived param 'range_3': %f", range_3);
        
    }

    if(ros::param::get("~exParamFile_dir_", exParamFile_dir_))
    {
        ROS_INFO("Retrived param 'exParamFile_dir_': %s", exParamFile_dir_.c_str());

    }

    // nh_.param("use_vox_filter", use_vox_filter_, false);
    // nh_.param("voxel_grid_size", voxel_grid_size_, 0.01);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh_, cloud1_tp, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh_, cloud2_tp, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud3_sub(nh_, cloud3_tp, 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),cloud1_sub, cloud2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    cloud_fused_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_fused_tp, 1);
    cloud_fused_ifiltered_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/fusion_cloud_ifiltered", 1);
    cloud_fused_i_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/fusion_cloud_i", 1);
    cloud_fused_zfiltered_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/fusion_cloud_zfiltered", 1);
    cloud1_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor1", 1);
    cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor2", 1);
    cloud3_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor3", 1);
    
    ostringstream os_in;
    os_in << exParamFile_dir_;

    Tr = load_exParam(os_in.str().c_str());
    cout << "load Tr = " << endl << Tr << endl;
    // Tr = Tr.inverse();


    ros::Rate loop_rate(100);
	while(ros::ok())
	{
        // ros::spinOnce();    // callback only once, this will do only once and then unsubscribe
        ros::spin();    // callback continuously
		loop_rate.sleep();
	}

    ros::shutdown();

	return 0;
}