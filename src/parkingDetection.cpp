#include "utility.h"
#include "lio_sam/cloud_info.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>

#include "lidarod/render/render.h"
#include "lidarod/processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "lidarod/processPointClouds.cpp"

// #include <pcl>


// struct OusterPointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )dectect

//或许这里需要设置radar的数据结构 但暂时先就用pcl的pointcloudxyz

// Use the Velodyne point format as a common representation
// using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class parkingDetection : public ParamServer
{
private:

    ros::Subscriber subMap;
    
    ros::Publisher pubParkinglot;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    sensor_msgs::PointCloud2 currentCloudMsg;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;  //点云输入

    double timeScanCur;  //对于激光 一帧点云有开始和结束两个时间 对于毫米波 我们只使用timeScanCur
    double timeScanEnd;  //这个之后一定不再使用
    std_msgs::Header cloudHeader;
    // boost::shared_ptr<pcl::visualization::CloudViewer> viewer;
    ros::Timer viewer_timer; 

    ProcessPointClouds<PointType> point_cloud_processor;


public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    parkingDetection()
    {
        subMap = nh.subscribe<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 5, &parkingDetection::mapHandler, this, ros::TransportHints().tcpNoDelay());
        
        // subMap = nh.subscribe<sensor_msgs::PointCloud2>("/radar_points2", 5, &parkingDetection::mapHandler, this, ros::TransportHints().tcpNoDelay());
        //这里的回调函数应该要改//TODO:

        pubParkinglot = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/mapping/submap", 1); //提取的点云，往外发，我们可能不需要这个

        allocateMemory();
        resetParameters();

        // viewer_timer = nh.createTimer(ros::Duration(0.1), &parkingDetection::freshVideo,this);

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        viewer.reset(new pcl::visualization::PCLVisualizer("Object Detection"));
        CameraAngle setAngle = FPS;
        initCamera(setAngle, viewer);
        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
    }

    ~parkingDetection(){}

    void mapHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) //TODO:
    {
        if (!cacheAndCutPointCloud(laserCloudMsg))
            return;

        objectDetection();
        
        publishClouds();
        
        resetParameters();

    }

    bool cacheAndCutPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg); //放进点云队列
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud  //点云covert
        currentCloudMsg = std::move(cloudQueue.front()); //拿到当前点云
        cloudQueue.pop_front();  //队列里pop一个
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn); //转成pcl消息 laserCloudIn
        
        // pcl::CropBox<PointType> boxFilter;

        // boxFilter.setMin(Eigen::Vector4f(-10, -6, -2.8, 1.0));

        // boxFilter.setMax(Eigen::Vector4f(6, 0, 0, 1.0));

        // boxFilter.setInputCloud(laserCloudIn);

        // boxFilter.filter(*laserCloudIn);

        // get timestamp 获取时间
        cloudHeader = currentCloudMsg.header;
        //手动做时延处理 这非常不合理 只是为了跑通代码
        // cloudHeader.stamp.sec -=2;
        timeScanCur = cloudHeader.stamp.toSec(); // TODO:权宜之计
        // timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        // timeScanEnd = timeScanCur +0.2; //这里我的想法是，让timeScanEnd等于下一帧的时间，现在获取这个可能会出错，所以暂时加入magic number
        timeScanEnd =timeScanCur;
  

        return true;
    }

    void objectDetection()
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        cityBlock(viewer, point_cloud_processor, laserCloudIn);

    }

    void freshVideo(const ros::TimerEvent&)
    {
        if(!viewer->wasStopped())
            {
                viewer->spinOnce(); 
                // ros::shutdown();
            }
    }


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_cloud_processor, pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
    renderPointCloud(viewer, input_cloud, "InputCloud");

    // Input point cloud, filter resolution, min Point, max Point
    constexpr float kFilterResolution = 0.2;
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);
    auto filter_cloud = point_cloud_processor.FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);
    // auto filter_cloud=input_cloud;

//    renderPointCloud(viewer, filter_cloud, "FilteredCloud");

    constexpr int kMaxIterations = 100;
    constexpr float kDistanceThreshold = 0.4;
    auto segment_cloud = point_cloud_processor.SegmentPlane(filter_cloud, kMaxIterations, kDistanceThreshold);

//    // render obstacles point cloud with red
   renderPointCloud(viewer, segment_cloud.first, "ObstacleCloud", Color(1, 0, 0));
    // render ground plane with green
   renderPointCloud(viewer, segment_cloud.second, "GroundCloud", Color(0, 1, 0));


    /*** Euclidean clustering ***/
    // float clusterTolerance, int minSize, int maxSize
    constexpr float kClusterTolerance = 0.35;
    constexpr int kMinSize = 15;
    constexpr int kMaxSize = 600;
    auto cloud_clusters = point_cloud_processor.Clustering(segment_cloud.first, kClusterTolerance, kMinSize, kMaxSize);

    int cluster_ID = 1;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    constexpr float kBBoxMinHeight = 0.75;
    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);

//        renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

        Box box = point_cloud_processor.BoundingBox(cluster);
        // Filter out some cluster with little points and shorter in height
        if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) {
            renderBox(viewer, box, cluster_ID);
        }

        cluster_ID++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->setBackgroundColor(0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle) {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem(1.0);
}
    
    //发布rangeMat
    void publishClouds()
    {
        sensor_msgs::PointCloud2 msg ;
        msg.header = cloudHeader;

        // pubParkinglot.publish(msg);

    }
    int conventHeight2Pixel(int heightlow , int heighthigh, float height)
    {
        return floor((height-heightlow)/(heighthigh-heightlow)*255);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    //打开DEBUG调试信息
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

    parkingDetection IP;
    
    ROS_INFO("\033[1;32m----> Parkinglot Detection Started.\033[0m");

    // ros::MultiThreadedSpinner spinner(3);
    while (IP.nh.ok())
    {
        if(!IP.viewer->wasStopped())
            {
                IP.viewer->spinOnce(); 
                // ros::shutdown();
                ros::spinOnce();
            }
    }
    
    // ros::spinOnce();
    // spinner.spin();
    
    return 0;
}