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

#include "render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "lidarod/processPointClouds.cpp"
#include "TRK4D_Geometry.h"



// #include <pcl>
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

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
    ros::Subscriber subLocate;
    
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

    Box parkinglotMemory;
    nav_msgs::Odometry currentLocate;
    nav_msgs::Odometry memoryLocate;


public:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    parkingDetection()
    {
        subMap = nh.subscribe<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 5, &parkingDetection::mapHandler, this, ros::TransportHints().tcpNoDelay());
        subLocate = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry",5,&parkingDetection::locateHandler, this, ros::TransportHints().tcpNoDelay());
        
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

    void locateHandler(const nav_msgs::OdometryConstPtr& odometryMsg) //TODO:
    {
        currentLocate = *odometryMsg;
    }


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
    const Eigen::Vector4f kMinPoint(-50, -6.0, -3, 1); //deafult (-50, -6.0, -3, 1);
    const Eigen::Vector4f kMaxPoint(60, 6.5, 4, 1);   //default (60, 6.5, 4, 1);
    auto filter_cloud = point_cloud_processor.FilterCloud(input_cloud, kFilterResolution, kMinPoint, kMaxPoint);
    // auto filter_cloud=input_cloud;

//    renderPointCloud(viewer, filter_cloud, "FilteredCloud");

    constexpr int kMaxIterations = 200;//default 100
    constexpr float kDistanceThreshold = 0.2; //default 0.2
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

    int cluster_ID = 2;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 0, 1), Color(0.5, 0, 1)};
    int num_of_colors = colors.size();

    Box host_box = {-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, host_box, 0, Color(0.5, 0, 1), 0.8);

    Box detect_box = {0, -6, -1, 1.7, -1.7, -0.4};
    renderBox(viewer, detect_box, 1, Color(0.5, 1, 0), 0.8);
    // vector<TRK4D_POINT3> bbox_vertex_a;
    TRK4D_POINT3 detect_box_trans[4];
    box2BboxVertex(detect_box,detect_box_trans);
    bool overlapFlag=0;

    constexpr float kBBoxMinHeight = 0.75;
    for(const auto& cluster : cloud_clusters) {
        std::cout << "cluster size ";
        point_cloud_processor.numPoints(cluster);

//        renderPointCloud(viewer, cluster, "ObstacleCloud" + std::to_string(cluster_ID), colors[cluster_ID % num_of_colors]);

        Box box = point_cloud_processor.BoundingBox(cluster);
        TRK4D_POINT3 object_box_trans[4];
        box2BboxVertex(box,object_box_trans);
        if (TRK4D_fn_isBBoxOverlap(detect_box_trans,object_box_trans))
        {
            if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) 
            {
                renderBox(viewer, box, cluster_ID,Color(0.5, 1, 0));
            }
            overlapFlag=1;
        }
        else{
            if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2)
            {
                renderBox(viewer, box, cluster_ID);
            }
        }
        // TRK4D_fn_isBBoxOverlap( detect_box_trans,object_box_trans);
        // Filter out some cluster with little points and shorter in height
        // if (box.z_max - box.z_min >= kBBoxMinHeight || cluster->points.size() >= kMinSize * 2) {
        //     renderBox(viewer, box, cluster_ID);
        // }

        cluster_ID++;
    }
    if(overlapFlag==0)//如果当前没有障碍物 那么停车位置于初始位置  
    {
        parkinglotMemory = detect_box;
        memoryLocate = currentLocate;
    }
    else //如果当前有障碍物，那么停车位的位置是memorylot由 memorylocate和currentlocate之间的变换而转换而来的
    {
        PointTypePose memoryPose6D = navodometry2PointTypePose(memoryLocate);
        PointTypePose currentPose6D = navodometry2PointTypePose(currentLocate);

        Eigen::Affine3f transMemory = pcl::getTransformation(memoryPose6D.x, memoryPose6D.y, memoryPose6D.z, memoryPose6D.roll, memoryPose6D.pitch, memoryPose6D.yaw);
        // transMemory = transMemory.inverse();
        Eigen::Affine3f transCurrent = pcl::getTransformation(currentPose6D.x, currentPose6D.y, currentPose6D.z, currentPose6D.roll, currentPose6D.pitch, currentPose6D.yaw);
        // transCurrent = transCurrent.inverse();
        Eigen::Affine3f transBetween =transCurrent.inverse()*transMemory;

        Box tempBox{100, 100, -1, -100, -100, 1};
        vector<TRK4D_POINT2> tempPoints;
        tempPoints.push_back({detect_box.x_max,detect_box.y_max});
        tempPoints.push_back({detect_box.x_max,detect_box.y_min});
        tempPoints.push_back({detect_box.x_min,detect_box.y_max});
        tempPoints.push_back({detect_box.x_min,detect_box.y_min});

        // vector<TRK4D_POINT2> resPoints(4);int indextemp=0;

        for (auto iter=tempPoints.cbegin(); iter != tempPoints.cend(); iter++)
        {
            float x =transBetween(0,0) * (*iter).x + transBetween(0,1) * (*iter).y + transBetween(0,2) * 1+ transBetween(0,3);
            float y =transBetween(1,0) * (*iter).x + transBetween(1,1) * (*iter).y + transBetween(1,2) * 1+ transBetween(1,3);
            tempBox.x_max = max(tempBox.x_max,x);
            tempBox.y_max = max(tempBox.y_max,y);
            tempBox.x_min = min(tempBox.x_min,x);
            tempBox.y_min = min(tempBox.y_min,y);
        }

        renderBox(viewer, tempBox, cluster_ID+1,Color(0, 1, 0));
        

    }
}

void computeTransedPoint(float x, float y)
{
    
}

PointTypePose navodometry2PointTypePose(nav_msgs::Odometry odometryIn)
{
    PointTypePose thisPose6D;
    thisPose6D.x = odometryIn.pose.pose.position.x;
    thisPose6D.y = odometryIn.pose.pose.position.y;
    thisPose6D.z = odometryIn.pose.pose.position.z;

    tf::Quaternion bt_q;
    quaternionMsgToTF(odometryIn.pose.pose.orientation, bt_q);
    tfScalar pitch, roll, yaw;
    tf::Matrix3x3(bt_q).getRPY( roll, pitch,yaw);

    thisPose6D.roll  = roll;
    thisPose6D.pitch = pitch;
    thisPose6D.yaw   = yaw;
    return thisPose6D;
}

void box2BboxVertex(Box inputbox,TRK4D_POINT3* result)
{
    result[0].x = inputbox.x_max;
    result[0].y = inputbox.y_max;
    result[0].z = 0;
    result[1].x = inputbox.x_min;
    result[1].y = inputbox.y_max;
    result[1].z = 0;
    result[2].x = inputbox.x_min;
    result[2].y = inputbox.y_min;
    result[2].z = 0;
    result[3].x = inputbox.x_max;
    result[3].y = inputbox.y_min;
    result[3].z = 0;


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