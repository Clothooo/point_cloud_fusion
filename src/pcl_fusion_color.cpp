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

#define DEBUG 1

using namespace std;
using namespace sensor_msgs;
using namespace Eigen;
using namespace message_filters;
using namespace pcl;
using namespace ros;

std::string cloud1_tp, cloud2_tp, cloud3_tp, cloud_ref_tp, cloud_fused_tp;
ros::Publisher cloud1_world_pub_, cloud_ref_pub_, cloud2_world_pub_, cloud3_world_pub_, 
    cloud_fused_pub_, cloud_fused_ifiltered_pub_, cloud_fused_i_pub_, cloud_fused_zfiltered_pub_;
ros::Publisher cloud1_world_rgb_pub_, cloud2_world_rgb_pub_, cloud3_world_rgb_pub_, cloud_fused_rgb_pub_;
std::string exParamFile_dir_ = "";

bool use_vox_filter_ = false;
double voxel_grid_size_ = 0.01;
double range_1 = 0.0, range_2 = 0.0, range_3 = 0.0;
Eigen::Matrix4d Tr;


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
void callback_rectify1(const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
    ROS_INFO("<<<<<<<<<<< 1 Point Cloud Rectify <<<<<<<<<<<");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        cloud_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    fromROSMsg(*msg_pc, *cloud_in);
    
    pcl::transformPointCloud(*cloud_in, *cloud_world, Tr);

    sensor_msgs::PointCloud2 cloud_world_ros;
    pcl::toROSMsg(*cloud_world, cloud_world_ros);
    cloud_world_ros.header = msg_pc->header;
    cloud_fused_pub_.publish(cloud_world_ros);
}

void callback_register2(const sensor_msgs::PointCloud2ConstPtr& msg_pc1, const sensor_msgs::PointCloud2ConstPtr& msg_pc_ref)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor1_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor2_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        fused_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor1_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor2_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        fused_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);                                        

    fromROSMsg(*msg_pc1, *sensor1_cloud);
    fromROSMsg(*msg_pc_ref, *sensor2_cloud);

    ROS_INFO("<<<<<<<<<<<<<<< 2 Point Cloud Register & Fusion <<<<<<<<<<<<<<<");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_world (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        cloud2_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*sensor1_cloud, *cloud1_world, Tr);
    pcl::copyPointCloud(*sensor2_cloud, *cloud2_world);
    *fused_cloud += *cloud1_world;
    *fused_cloud += *cloud2_world;

    // coloring...
    pcl::copyPointCloud(*cloud1_world, *sensor1_cloud_rgb);
    pcl::copyPointCloud(*cloud2_world, *sensor2_cloud_rgb);
    for(auto p = sensor1_cloud_rgb->points.begin(); p < sensor1_cloud_rgb->points.end(); ++p)
    {
        p->r = 255;
        p->g = 0;
        p->b = 0;
    }
    for(auto p = sensor2_cloud_rgb->points.begin(); p < sensor2_cloud_rgb->points.end(); ++p)
    {
        p->r = 255;
        p->g = 255;
        p->b = 0;
    }
    *fused_cloud_rgb += *sensor1_cloud_rgb;
    *fused_cloud_rgb += *sensor2_cloud_rgb;


    sensor_msgs::PointCloud2 cloud1_world_ros;
    pcl::toROSMsg(*cloud1_world, cloud1_world_ros);
    cloud1_world_ros.header = msg_pc_ref->header;
    cloud1_world_pub_.publish(cloud1_world_ros);

    sensor_msgs::PointCloud2 cloud2_world_ros;
    pcl::toROSMsg(*cloud2_world, cloud2_world_ros);
    cloud2_world_ros.header = msg_pc_ref->header;
    cloud_ref_pub_.publish(cloud2_world_ros);

    // ********************** 2. Downsampling *********************
    if(use_vox_filter_)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(fused_cloud);
        sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
        sor.filter(*temp_cloud);

        cout << "PointCloud size after filter: " << temp_cloud->points.size() << endl;
        pcl::copyPointCloud(*temp_cloud, *fused_cloud);
    }
   
    sensor_msgs::PointCloud2 fused_cloud_ros;
    pcl::toROSMsg(*fused_cloud, fused_cloud_ros);
    fused_cloud_ros.header = msg_pc_ref->header;
    cloud_fused_pub_.publish(fused_cloud_ros);

    sensor_msgs::PointCloud2 fused_cloud_rgb_ros;
    pcl::toROSMsg(*fused_cloud_rgb, fused_cloud_rgb_ros);
    fused_cloud_rgb_ros.header = msg_pc_ref->header;
    cloud_fused_rgb_pub_.publish(fused_cloud_rgb_ros);


    return;
}

void callback_fusion3(const sensor_msgs::PointCloud2ConstPtr& msg_pc1, const sensor_msgs::PointCloud2ConstPtr& msg_pc2, const sensor_msgs::PointCloud2ConstPtr& msg_pc3)
{
    ROS_INFO("<<<<<<<<<< 3 Point Cloud Fusion <<<<<<<<<<");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor1_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor2_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor3_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        fused_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor1_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor2_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        sensor3_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        fused_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);

    fromROSMsg(*msg_pc1, *sensor1_cloud);
    fromROSMsg(*msg_pc2, *sensor2_cloud);
    fromROSMsg(*msg_pc3, *sensor3_cloud);

    *fused_cloud += *sensor1_cloud;
    *fused_cloud += *sensor2_cloud;
    *fused_cloud += *sensor3_cloud;

    if(use_vox_filter_)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(fused_cloud);
        sor.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
        sor.filter(*temp_cloud);

        cout << "PointCloud size after filter: " << temp_cloud->points.size() << endl;
        pcl::copyPointCloud(*temp_cloud, *fused_cloud);
    }

    // coloring....
    pcl::copyPointCloud(*sensor1_cloud, *sensor1_cloud_rgb);
    pcl::copyPointCloud(*sensor2_cloud, *sensor2_cloud_rgb);
    pcl::copyPointCloud(*sensor3_cloud, *sensor3_cloud_rgb);
    for(auto p = sensor1_cloud_rgb->points.begin(); p < sensor1_cloud_rgb->points.end(); ++p)
    {
        p->r = 255;
        p->g = 0;
        p->b = 0;
    }
    for(auto p = sensor2_cloud_rgb->points.begin(); p < sensor2_cloud_rgb->points.end(); ++p)
    {
        p->r = 255;
        p->g = 255;
        p->b = 0;
    }
    for(auto p = sensor3_cloud_rgb->points.begin(); p < sensor3_cloud_rgb->points.end(); ++p)
    {
        p->r = 0;
        p->g = 255;
        p->b = 255;
    }
    *fused_cloud_rgb += *sensor1_cloud_rgb;
    *fused_cloud_rgb += *sensor2_cloud_rgb;
    *fused_cloud_rgb += *sensor3_cloud_rgb;


    sensor_msgs::PointCloud2 cloud1_world_ros;
    pcl::toROSMsg(*sensor1_cloud, cloud1_world_ros);
    cloud1_world_ros.header = msg_pc1->header;
    cloud1_world_pub_.publish(cloud1_world_ros);

    sensor_msgs::PointCloud2 cloud2_world_ros;
    pcl::toROSMsg(*sensor2_cloud, cloud2_world_ros);
    cloud2_world_ros.header = msg_pc1->header;
    cloud2_world_pub_.publish(cloud2_world_ros);
    
    sensor_msgs::PointCloud2 cloud3_world_ros;
    pcl::toROSMsg(*sensor3_cloud, cloud3_world_ros);
    cloud3_world_ros.header = msg_pc1->header;
    cloud3_world_pub_.publish(cloud3_world_ros);
    
    sensor_msgs::PointCloud2 fused_cloud_ros;
    pcl::toROSMsg(*fused_cloud, fused_cloud_ros);
    fused_cloud_ros.header = msg_pc1->header;
    cloud_fused_pub_.publish(fused_cloud_ros);

    // rgb publish
    // sensor_msgs::PointCloud2 cloud1_world_rgb_ros;
    // pcl::toROSMsg(*sensor1_cloud_rgb, cloud1_world_rgb_ros);
    // cloud1_world_rgb_ros.header = msg_pc1->header;
    // cloud1_world_rgb_pub_.publish(cloud1_world_rgb_ros);

    // sensor_msgs::PointCloud2 cloud2_world_rgb_ros;
    // pcl::toROSMsg(*sensor2_cloud_rgb, cloud2_world_rgb_ros);
    // cloud2_world_rgb_ros.header = msg_pc1->header;
    // cloud2_world_rgb_pub_.publish(cloud2_world_rgb_ros);
    
    // sensor_msgs::PointCloud2 cloud3_world_rgb_ros;
    // pcl::toROSMsg(*sensor3_cloud_rgb, cloud3_world_rgb_ros);
    // cloud3_world_rgb_ros.header = msg_pc1->header;
    // cloud3_world_rgb_pub_.publish(cloud3_world_rgb_ros);
    
    sensor_msgs::PointCloud2 fused_cloud_rgb_ros;
    pcl::toROSMsg(*fused_cloud_rgb, fused_cloud_rgb_ros);
    fused_cloud_rgb_ros.header = msg_pc1->header;
    cloud_fused_rgb_pub_.publish(fused_cloud_rgb_ros);

}

void loadParam()
{
    if(ros::param::get("~cloud1_tp", cloud1_tp))
    {
        ROS_INFO("Retrived param 'cloud1_tp': %s", cloud1_tp.c_str());
    }
    if(ros::param::get("~cloud2_tp", cloud2_tp))
    {
        ROS_INFO("Retrived param 'cloud2_tp': %s", cloud2_tp.c_str());
    }
    if(ros::param::get("~cloud3_tp", cloud3_tp))
    {
        ROS_INFO("Retrived param 'cloud3_tp': %s", cloud3_tp.c_str());
    }
     if(ros::param::get("~cloud_ref_tp", cloud_ref_tp))
    {
        ROS_INFO("Retrived param 'cloud_ref_tp': %s", cloud_ref_tp.c_str());
    }
    if(ros::param::get("~cloud_fused_tp", cloud_fused_tp))
    {
        ROS_INFO("Retrived param 'cloud_fused_tp': %s", cloud_fused_tp.c_str());
    }
    if(ros::param::get("~use_vox_filter", use_vox_filter_)
    && ros::param::get("~voxel_grid_size", voxel_grid_size_))
    {
        ROS_INFO("Retrived param 'use_vox_filter': %d", use_vox_filter_);
        ROS_INFO("Retrived param 'voxel_grid_size': %f", voxel_grid_size_);   
    }
    if(ros::param::get("~exParamFile_dir_", exParamFile_dir_))
    {
        ROS_INFO("Retrived param 'exParamFile_dir_': %s", exParamFile_dir_.c_str());
        ostringstream os_in;
        os_in << exParamFile_dir_;
        Tr = load_exParam(os_in.str().c_str());
        cout << "load Tr = " << endl << Tr << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_fusion_real");
    ros::NodeHandle nh_("~");

    loadParam();
    string cloud_fused_rgb_tp = cloud_fused_tp + "_rgb";

    ros::Subscriber sub = nh_.subscribe("cloud_in", 1, callback_rectify1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud1_sub(nh_, cloud1_tp, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub(nh_, cloud2_tp, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud3_sub(nh_, cloud3_tp, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_ref_sub(nh_, cloud_ref_tp, 1);

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy_2pc;
    Synchronizer<MySyncPolicy_2pc> sync_2pc(MySyncPolicy_2pc(10),cloud1_sub, cloud_ref_sub);
    sync_2pc.registerCallback(boost::bind(&callback_register2, _1, _2));

    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy_3pc;
    Synchronizer<MySyncPolicy_3pc> sync_3pc(MySyncPolicy_3pc(10),cloud1_sub, cloud2_sub, cloud3_sub);
    sync_3pc.registerCallback(boost::bind(&callback_fusion3, _1, _2, _3));

    cloud_fused_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_fused_tp, 1);
    cloud_fused_ifiltered_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/fusion_cloud_ifiltered", 1);
    cloud_ref_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor_ref", 1);
    cloud1_world_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor1", 1);
    cloud2_world_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor2", 1);
    cloud3_world_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor3", 1);
    // cloud1_world_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor1_rgb", 1);
    // cloud2_world_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor2_rgb", 1);
    // cloud3_world_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("world_frame_pcl/sensor3_rgb", 1);
    cloud_fused_rgb_pub_ = nh_.advertise<sensor_msgs::PointCloud2> (cloud_fused_rgb_tp, 1);


    ros::Rate loop_rate(100);
	while(ros::ok())
	{
        ros::spinOnce();    // callback_register2 only once, this will do only once and then unsubscribe
        // ros::spin();    // callback_register2 continuously
		loop_rate.sleep();
	}

    ros::shutdown();

	return 0;
}