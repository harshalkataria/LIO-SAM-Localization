#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "color_print.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"
#include "tic_toc.h"
#include "utility.h"

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;  // GPS pose
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

enum InitializedFlag { NonInitialized, Initializing, Initialized, MayLost };
typedef pcl::PointCloud<PointType> CLOUD;
typedef CLOUD::Ptr CLOUD_PTR;
/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                     // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(
                                                     float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;
struct pcdmap {
    std::vector<CLOUD_PTR> corner_keyframes_;
    std::vector<CLOUD_PTR> surf_keyframes_;
    CLOUD_PTR globalMapCloud_;
    CLOUD_PTR cloudKeyPoses3D_;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D_;
    CLOUD_PTR globalCornerMapCloud_;
    CLOUD_PTR globalSurfMapCloud_;
    pcdmap() {
        globalMapCloud_.reset(new CLOUD);
        cloudKeyPoses3D_.reset(new CLOUD);
        globalCornerMapCloud_.reset(new CLOUD);
        globalSurfMapCloud_.reset(new CLOUD);
        cloudKeyPoses6D_.reset(new pcl::PointCloud<PointTypePose>());
    }
};

// EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// struct LidarFrame {
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     CLOUD_PTR laserCloud;
//     CLOUD_PTR corner;
//     CLOUD_PTR surf;
//     // IMUIntegrator imuIntegrator;
//     Eigen::Vector3d P;
//     Eigen::Vector3d V;
//     Eigen::Quaterniond Q;
//     Eigen::Vector3d bg;
//     Eigen::Vector3d ba;
//     double timeStamp;
//     LidarFrame() {
//         corner.reset(new CLOUD);
//         surf.reset(new CLOUD);
//         laserCloud.reset(new CLOUD());
//         P.setZero();
//         V.setZero();
//         Q.setIdentity();
//         bg.setZero();
//         ba.setZero();
//         timeStamp = 0;
//     }
// };

class mapOptimizationLocalization : public ParamServer {
   public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2* isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    std::string filename = std::string("/home/harshal/PARA/Projects/TII_RSE/Assignment/maps/res-0-highbay-tracking-test-10-minute-start-30-sec");
    pcdmap map;
    pcl::VoxelGrid<PointType> ds_corner_;
    pcl::VoxelGrid<PointType> ds_surf_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_keyposes_3d_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_map;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_map;

    CLOUD_PTR surround_surf;
    CLOUD_PTR surround_corner;

    Eigen::Matrix3d delta_Rl = Eigen::Matrix3d::Identity();
    Eigen::Vector3d delta_tl = Eigen::Vector3d::Zero();

    static const int localMapWindowSize = 30;
    int localMapID = 0;
    pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromLocal;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromLocal;
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;  //  publish laser to map tf
    ros::Subscriber sub_initial_pose_;
    nav_msgs::Path laserOdoPath;
    ros::Publisher pubLaserOdometryPath_;

    ros::Publisher pub_surf_;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubMapKeyPoses;
    ros::Publisher pubGLobalMapCloud;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;

    ros::ServiceServer srvSaveMap;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;    // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;      // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;    // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    std::map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of scan-to-map optimization

    // For loading map
    pcl::PointCloud<PointType>::Ptr _mapKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr _mapKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr _mapCornerCloud;
    pcl::PointCloud<PointType>::Ptr _mapSurfCloud;
    pcl::PointCloud<PointType>::Ptr _mapCloud;
    pcl::PointCloud<PointType>::Ptr _laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr _laserCloudSurfFromMapDS;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr _kdtreeSurfFromMap;

    PointTypePose _initpose;
    InitializedFlag initializedFlag;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    std::map<int, int> loopIndexContainer;  // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    mapOptimizationLocalization() {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);
        // nh.param<std::string>("location/filedir", filename, "");

        // sub_initial_pose_ =
        //     nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &mapOptimizationLocalization::initialPoseCB, this);
        pub_surf_ = nh.advertise<sensor_msgs::PointCloud2>("/surf_registered", 1);
        pubLaserOdometryPath_ = nh.advertise<nav_msgs::Path>("/path_mapped", 5);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);
        pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        pubMapKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_key_poses", 1);
        pubGLobalMapCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/global_map", 1);

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimizationLocalization::laserCloudInfoHandler, this,
                                                     ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry>(gpsTopic, 200, &mapOptimizationLocalization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimizationLocalization::loopInfoHandler, this,
                                                            ros::TransportHints().tcpNoDelay());

        srvSaveMap = nh.advertiseService("lio_sam/save_map", &mapOptimizationLocalization::saveMapService, this);

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        pubSLAMInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);
        ds_corner_.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        ds_surf_.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity,
                                                      surroundingKeyframeDensity);  // for surrounding key poses of scan-to-map optimization

        allocateMemory();

        bool _loadMap = true;
        std::string loadMapDirectory =
            std::string("/home/harshal/PARA/Projects/TII_RSE/Assignment/maps/highbay-tracking-test-10-minute-start-30-sec");
        if (_loadMap) {
            // if (!loadMapService(loadMapDirectory)) {
            //     ROS_ERROR("Cannot load map from %s", loadMapDirectory.c_str());
            // }
            if (loadmap())
                std::cout << ANSI_COLOR_GREEN << "load map successful..." << ANSI_COLOR_RESET << std::endl;
            else {
                std::cout << ANSI_COLOR_RED_BOLD << "WARN: load map failed." << ANSI_COLOR_RESET << std::endl;
                return;
            }
        }
        surround_surf.reset(new CLOUD);
        surround_corner.reset(new CLOUD);

        kdtree_keyposes_3d_.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_keyposes_3d_->setInputCloud(map.cloudKeyPoses3D_);  // init 3d-pose kdtree

        kdtree_corner_map.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_corner_map->setInputCloud(map.globalCornerMapCloud_);
        kdtree_surf_map.reset(new pcl::KdTreeFLANN<PointType>());
        kdtree_surf_map->setInputCloud(map.globalSurfMapCloud_);
        initializedFlag = NonInitialized;

        for (int i = 0; i < localMapWindowSize; i++) {
            localCornerMap[i].reset(new pcl::PointCloud<PointType>);
            localSurfMap[i].reset(new pcl::PointCloud<PointType>);
        }
        laserCloudCornerFromLocal.reset(new pcl::PointCloud<PointType>);
        laserCloudSurfFromLocal.reset(new pcl::PointCloud<PointType>);
    }

    void allocateMemory() {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());    // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());      // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());    // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        _mapKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        _mapKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        _mapCornerCloud.reset(new pcl::PointCloud<PointType>());
        _mapSurfCloud.reset(new pcl::PointCloud<PointType>());
        _mapCloud.reset(new pcl::PointCloud<PointType>());
        _laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        _laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());
        _kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        _kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn) {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
        publishLoadedMapAndKeyPoses();

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        // CLOUD_PTR laserCloudNormal;
        // laserCloudNormal.reset(new CLOUD());
        // for (int i = 0; i < laserCloudSurfLast->size(); i++) {
        //     PointTypeNormal p;
        //     p.x = laserCloudSurfLast->points[i].x;
        //     p.y = laserCloudSurfLast->points[i].y;
        //     p.z = laserCloudSurfLast->points[i].z;
        //     p.intensity = laserCloudSurfLast->points[i].intensity;
        //     p.normal_x = 0.0;
        //     p.normal_y = 0.0;
        //     p.normal_z = 0.0;
        //     laserCloudNormal->push_back(p);
        // }
        // std::cout << ANSI_COLOR_BLUE << "laserCloudSurfLast Header: " << msgIn->cloud_surface.header.frame_id << ANSI_COLOR_RESET << std::endl;

        // Eigen::Vector3d P;
        // Eigen::Quaterniond Q;
        // Eigen::Matrix4d transformLastMapped = Eigen::Matrix4d::Identity();

        // //  predict pose use constant velocity
        // P = transformLastMapped.topLeftCorner(3, 3) * delta_tl + transformLastMapped.topRightCorner(3, 1);
        // Eigen::Matrix3d m3d = transformLastMapped.topLeftCorner(3, 3) * delta_Rl;
        // Q = m3d;
        // if (initializedFlag == Initializing) {
        //     if (ICPScanMatchGlobal(laserCloudNormal, P, Q)) {
        //         initializedFlag = Initialized;
        //         std::cout << ANSI_COLOR_GREEN << "icp scan match successful ..." << ANSI_COLOR_RESET << std::endl;
        //     }

        //     transformLastMapped.topLeftCorner(3, 3) = Q.toRotationMatrix();
        //     transformLastMapped.topRightCorner(3, 1) = P;

        //     pubOdometry(transformLastMapped, timeLaserInfoCur);
        // } else if (initializedFlag == Initialized) {
        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimizationLocalization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }
        // }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg) { gpsQueue.push_back(*gpsMsg); }

    void pointAssociateToMap(PointType const* const pi, PointType* const po) {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z +
                transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z +
                transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z +
                transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn) {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur =
            pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i) {
            const auto& pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[]) {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }
    bool loadmap() {
        std::cout << ANSI_COLOR_YELLOW << "file dir: " << filename << ANSI_COLOR_RESET << std::endl;
        CLOUD_PTR globalCornerCloud(new CLOUD);
        CLOUD_PTR globalSurfCloud(new CLOUD);

        std::string fn_poses_ = filename + "/trajectory.pcd";
        std::string fn_corner_ = filename + "/CornerMap.pcd";
        std::string fn_surf_ = filename + "/SurfMap.pcd";
        std::string fn_global_ = filename + "/GlobalMap.pcd";
        std::string fn_transformations_ = filename + "/transformations.pcd";

        if (pcl::io::loadPCDFile(fn_transformations_, *map.cloudKeyPoses6D_) == -1 || pcl::io::loadPCDFile(fn_poses_, *map.cloudKeyPoses3D_) == -1 ||
            pcl::io::loadPCDFile(fn_corner_, *globalCornerCloud) == -1 || pcl::io::loadPCDFile(fn_surf_, *globalSurfCloud) == -1 ||
            pcl::io::loadPCDFile(fn_global_, *map.globalMapCloud_)) {
            std::cout << ANSI_COLOR_RED << "couldn't load pcd file" << ANSI_COLOR_RESET << std::endl;
            return false;
        }
        std::cout << ANSI_COLOR_YELLOW << "load pcd file successful" << ANSI_COLOR_RESET << std::endl;
        map.corner_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
        map.surf_keyframes_.resize(map.cloudKeyPoses3D_->points.size());
        for (int i = 0; i < map.cloudKeyPoses3D_->points.size(); ++i) {
            // std::cout << ANSI_COLOR_YELLOW << "key pose: " << i << " " << map.cloudKeyPoses3D_->points[i].x << " "
            //           << map.cloudKeyPoses3D_->points[i].y << " " << map.cloudKeyPoses3D_->points[i].z << " "
            //           << map.cloudKeyPoses3D_->points[i].intensity << ANSI_COLOR_RESET << std::endl;
            map.corner_keyframes_[i] = CLOUD_PTR(new CLOUD);
            map.surf_keyframes_[i] = CLOUD_PTR(new CLOUD);
        }
        std::cout << ANSI_COLOR_YELLOW << " keyframes initiated " << ANSI_COLOR_RESET << std::endl;
        for (int i = 0; i < globalCornerCloud->points.size(); ++i) {
            const auto& p = globalCornerCloud->points[i];
            map.corner_keyframes_[p.intensity]->points.push_back(p);
            // std::cout << ANSI_COLOR_CYAN << "corner keyframe: " << i << " " << p.intensity << " " << p.x << " " << p.y << " " << p.z
            //           << ANSI_COLOR_RESET << std::endl;
        }
        std::cout << ANSI_COLOR_YELLOW << "corner keyframes loaded " << ANSI_COLOR_RESET << std::endl;
        for (int i = 0; i < globalSurfCloud->points.size(); ++i) {
            const auto& p = globalSurfCloud->points[i];
            map.surf_keyframes_[p.intensity]->points.push_back(p);
            // std::cout << ANSI_COLOR_CYAN << "surf keyframe: " << int(p.intensity) << " " << p.x << " " << p.y << " " << p.z << ANSI_COLOR_RESET
            //   << std::endl;
        }
        std::cout << ANSI_COLOR_YELLOW << "surf keyframes loaded " << ANSI_COLOR_RESET << std::endl;
        // for (int i = 0; i < map.cloudKeyPoses3D_->points.size(); ++i) {
        //     PointTypePose thisPose6D;
        //     // Take inverse of thisPose6D
        //     thisPose6D.roll = -map.cloudKeyPoses6D_->points[i].roll;
        //     thisPose6D.pitch = -map.cloudKeyPoses6D_->points[i].pitch;
        //     thisPose6D.yaw = -map.cloudKeyPoses6D_->points[i].yaw;
        //     thisPose6D.x = -map.cloudKeyPoses6D_->points[i].x;
        //     thisPose6D.y = -map.cloudKeyPoses6D_->points[i].y;
        //     thisPose6D.z = -map.cloudKeyPoses6D_->points[i].z;
        //     thisPose6D.intensity = map.cloudKeyPoses6D_->points[i].intensity;
        //     thisPose6D.time = map.cloudKeyPoses6D_->points[i].time;
        //     *map.corner_keyframes_[i] = *TransformPointCloud(map.corner_keyframes_[i], &thisPose6D);
        //     *map.surf_keyframes_[i] = *TransformPointCloud(map.surf_keyframes_[i], &thisPose6D);
        // }
        std::cout << ANSI_COLOR_YELLOW << "keyframes transformed " << ANSI_COLOR_RESET << std::endl;
        ds_corner_.setInputCloud(globalCornerCloud);
        ds_corner_.filter(*map.globalCornerMapCloud_);

        long unsigned int totalKeyFramesPointsNum = 0;
        for (int i = 0; i < map.cloudKeyPoses3D_->points.size(); ++i) {
            totalKeyFramesPointsNum += map.corner_keyframes_[i]->points.size() + map.surf_keyframes_[i]->points.size();
        }
        std::cout << ANSI_COLOR_GREEN << "cloudKeyPoses3DSize: " << map.cloudKeyPoses3D_->points.size()
                  << ", globalCornerCloudSize: " << globalCornerCloud->points.size()
                  << ", globalSurfCloudSize: " << map.globalCornerMapCloud_->points.size() << ", totalKeyFramePointsNum: " << totalKeyFramesPointsNum
                  << ANSI_COLOR_RESET << std::endl;
        ds_surf_.setInputCloud(globalSurfCloud);
        ds_surf_.filter(*map.globalSurfMapCloud_);
        return true;
    }

    bool loadMapService(const std::string& loadMapDirectory) {
        // Load each file into its corresponding point cloud
        if (pcl::io::loadPCDFile(loadMapDirectory + "/trajectory.pcd", *_mapKeyPoses3D) == -1) {
            std::cerr << "Failed to load map key poses 3D file: " << loadMapDirectory + "/trajectory.pcd" << std::endl;
            return false;
        }
        if (pcl::io::loadPCDFile(loadMapDirectory + "/transformations.pcd", *_mapKeyPoses6D) == -1) {
            std::cerr << "Failed to load map key poses 6D file: " << loadMapDirectory + "/transformations.pcd" << std::endl;
            return false;
        }
        if (pcl::io::loadPCDFile(loadMapDirectory + "/CornerMap.pcd", *_mapCornerCloud) == -1) {
            std::cerr << "Failed to load global corner cloud file: " << loadMapDirectory + "/CornerMap.pcd" << std::endl;
            return false;
        }
        if (pcl::io::loadPCDFile(loadMapDirectory + "/SurfMap.pcd", *_mapSurfCloud) == -1) {
            std::cerr << "Failed to load global surf cloud file: " << loadMapDirectory + "/SurfMap.pcd" << std::endl;
            return false;
        }
        if (pcl::io::loadPCDFile(loadMapDirectory + "/GlobalMap.pcd", *_mapCloud) == -1) {
            std::cerr << "Failed to load global map cloud file: " << loadMapDirectory + "/GlobalMap.pcd" << std::endl;
            return false;
        }
        // Create kd-tree for the downsampled global maps
        // _kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        // _kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        // Print success message
        std::cout << "Loaded map from " << loadMapDirectory << std::endl;

        return true;
    }
    // Function to publish the loaded maps and keyPoses on different ros topics
    void publishLoadedMapAndKeyPoses() {
        // Publish the loaded map key poses
        if (pubMapKeyPoses.getNumSubscribers() != 0) {
            publishCloud(pubMapKeyPoses, map.cloudKeyPoses3D_, timeLaserInfoStamp, mapFrame);
        }
        // Publish the loaded global map
        if (pubGLobalMapCloud.getNumSubscribers() != 0) {
            // publishCloud(pubGLobalMapCloud, map.globalMapCloud_, timeLaserInfoStamp, mapFrame);
        }
    }

    void initialPoseCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        // publishLoadedMapAndKeyPoses();
        //  低频，不加锁
        PointType p;

        _initpose.x = msg->pose.pose.position.x;
        _initpose.y = msg->pose.pose.position.y;
        _initpose.z = msg->pose.pose.position.z;
        double roll, pitch, yaw;
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        _initpose.roll = roll;
        _initpose.pitch = pitch;
        _initpose.yaw = yaw;

        p.x = _initpose.x;
        p.y = _initpose.y;
        p.z = _initpose.z;
        std::cout << "Get initial pose: " << _initpose.x << " " << _initpose.y << " " << _initpose.z << " " << roll << " " << pitch << " " << yaw
                  << std::endl;
        bool extractSurroundKeyFrameFlag = extractSurroundKeyFrames(p);
        if (extractSurroundKeyFrameFlag) {
            std::cout << "extractSurroundKeyFrames successful" << std::endl;
        } else {
            std::cout << "extractSurroundKeyFrames failed" << std::endl;
        }
        std::cout << "Change flag from " << initializedFlag << " to " << Initializing << ", start do localizating ..." << std::endl;
        if (initializedFlag != NonInitialized) {  // TODO: 非第一次执行，需要重置部分参数
            delta_Rl = Eigen::Matrix3d::Identity();
            delta_tl = Eigen::Vector3d::Zero();
            for (int i = 0; i < localMapWindowSize; i++) {
                localCornerMap[i]->clear();
                localSurfMap[i]->clear();
            }
            localMapID = 0;
            laserCloudCornerFromLocal->clear();
            laserCloudSurfFromLocal->clear();
            std::cout << ANSI_COLOR_YELLOW << " Cleared laser Cloud Corner from local and surf from local" << ANSI_COLOR_RESET << std::endl;
        }
        initializedFlag = Initializing;
    }

    bool extractSurroundKeyFrames(const PointType& p) {
        TicToc tc;
        tc.tic();
        std::cout << "-----extract surround keyframes ------ " << std::endl;
        try {
            std::vector<int> point_search_idx_;
            std::vector<float> point_search_dist_;
            double surround_search_radius_ = 5.0;
            kdtree_keyposes_3d_->radiusSearch(p, surround_search_radius_, point_search_idx_, point_search_dist_, 0);
            surround_surf->clear();
            surround_corner->clear();
            for (int i = 0; i < point_search_idx_.size(); ++i) {
                *surround_surf += *map.surf_keyframes_[point_search_idx_[i]];
                *surround_corner += *map.corner_keyframes_[point_search_idx_[i]];
            }
            ds_corner_.setInputCloud(surround_corner);
            ds_corner_.filter(*surround_corner);
            ds_surf_.setInputCloud(surround_surf);
            ds_surf_.filter(*surround_surf);
        } catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
            return false;
        }
        double tt = tc.toc();
        std::cout << "extractSurroundKeyFrames takes: " << tt << "ms" << std::endl;
        return true;
    }
    CLOUD_PTR TransformPointCloud(CLOUD_PTR cloudIn, PointTypePose* transformIn) {
        CLOUD_PTR cloudOut(new CLOUD());
        PointType* pointfrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);
        for (int i = 0; i < cloudSize; i++) {
            pointfrom = &cloudIn->points[i];
            float x1 = pointfrom->x;
            float y1 = cos(transformIn->roll) * pointfrom->y - sin(transformIn->roll) * pointfrom->z;
            float z1 = sin(transformIn->roll) * pointfrom->y + cos(transformIn->roll) * pointfrom->z;

            float x2 = cos(transformIn->pitch) * x1 + sin(transformIn->pitch) * z1;
            float y2 = y1;
            float z2 = -sin(transformIn->pitch) * x1 + cos(transformIn->pitch) * z1;

            pointTo.x = cos(transformIn->yaw) * x2 - sin(transformIn->yaw) * y2 + transformIn->x;
            pointTo.y = sin(transformIn->yaw) * x2 + cos(transformIn->yaw) * y2 + transformIn->y;
            pointTo.z = z2 + transformIn->z;
            pointTo.intensity = pointfrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    bool ICPScanMatchGlobal(CLOUD_PTR kframeListLaserCloud, Eigen::Vector3d& P, Eigen::Quaterniond& Q) {
        if (kframeListLaserCloud->empty()) std::cout << "Pass one lidar frame" << std::endl;

        CLOUD_PTR curr_surf(new CLOUD());
        std::cout << ANSI_COLOR_WHITE << "surf size: " << kframeListLaserCloud->points.size() << ANSI_COLOR_RESET << std::endl;
        for (const auto& p : kframeListLaserCloud->points) {
            curr_surf->push_back(p);
        }
        // ds_surf_.setInputCloud(surf);
        // ds_surf_.filter(*surf);

        TicToc tc;
        tc.tic();
        CLOUD_PTR cloud_icp(new CLOUD());
        // *cloud_icp += *TransformPointCloud(corner, &initpose);
        // *cloud_icp += *TransformPointCloud(curr_surf, &_initpose);
        std::cout << ANSI_COLOR_WHITE << "surround_surf size: " << surround_surf->points.size() << ANSI_COLOR_RESET << std::endl;

        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(200);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-2);
        icp.setEuclideanFitnessEpsilon(1e-2);
        icp.setRANSACIterations(0);

        icp.setInputSource(cloud_icp);
        icp.setInputTarget(surround_surf);
        CLOUD_PTR unused_result(new CLOUD());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > 0.4) {
            std::cout << ANSI_COLOR_RED << "initial loc failed...,score: " << icp.getFitnessScore() << ANSI_COLOR_RESET << std::endl;
            return false;
        }
        std::cout << ANSI_COLOR_YELLOW << " icp has converged, score: " << icp.getFitnessScore() << ANSI_COLOR_RESET << std::endl;
        Eigen::Affine3f correct_transform;
        correct_transform = icp.getFinalTransformation();
        Eigen::Matrix4d curr_pose = toMatrix(_initpose);

        Eigen::Matrix4d pose = correct_transform.matrix().cast<double>() * curr_pose;

        Q = pose.block<3, 3>(0, 0);
        P = pose.topRightCorner(3, 1);  //  update pose here

        double tt = tc.toc();
        std::cout << "icp takes: " << tt << "ms" << std::endl;
        CLOUD_PTR output(new CLOUD);

        pcl::transformPointCloud(*curr_surf, *output, pose);
        sensor_msgs::PointCloud2 msg_target;
        // pcl::toROSMsg(*cloud_icp, msg_target);
        pcl::toROSMsg(*output, msg_target);
        msg_target.header.stamp = ros::Time::now();
        msg_target.header.frame_id = "world";
        pub_surf_.publish(msg_target);

        return true;
    }

    Eigen::Matrix4d toMatrix(PointTypePose& p) {
        Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(p.roll, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(p.yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond rotation = yawAngle * pitchAngle * rollAngle;
        odom.block(0, 0, 3, 3) = rotation.toRotationMatrix();
        odom(0, 3) = p.x, odom(1, 3) = p.y, odom(2, 3) = p.z;
        return odom;
    }

    void pubOdometry(Eigen::Matrix4d pose, double& time) {
        Eigen::Quaterniond Q(pose.block<3, 3>(0, 0));
        ros::Time ros_time = ros::Time().fromSec(time);
        transform_.stamp_ = ros_time;
        transform_.setRotation(tf::Quaternion(Q.x(), Q.y(), Q.z(), Q.w()));
        transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        transform_.frame_id_ = "world";
        transform_.child_frame_id_ = "base_link";
        broadcaster_.sendTransform(transform_);

        geometry_msgs::PoseStamped laserPose;
        laserPose.header.frame_id = "world";
        laserPose.header.stamp = ros_time;

        laserPose.pose.orientation.x = Q.x();
        laserPose.pose.orientation.y = Q.y();
        laserPose.pose.orientation.z = Q.z();
        laserPose.pose.orientation.w = Q.w();
        laserPose.pose.position.x = pose(0, 3);
        laserPose.pose.position.y = pose(1, 3);
        laserPose.pose.position.z = pose(2, 3);

        laserOdoPath.header.stamp = ros_time;
        laserOdoPath.poses.push_back(laserPose);
        laserOdoPath.header.frame_id = "/world";
        pubLaserOdometryPath_.publish(laserOdoPath);
    }

    bool saveMapService(lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res) {
        string saveMapDirectory;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        if (req.destination.empty())
            saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
        else
            saveMapDirectory = std::getenv("HOME") + req.destination;
        cout << "Save destination: " << saveMapDirectory << endl;
        // create directory and remove old files;
        int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
        unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());
        // save key frame transformations
        pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            for (int j = 0; j < cornerCloudKeyFrames[i]->points.size(); j++) {
                std::cout << " cornerCloudKeyFrame: " << i << " point: " << j << " intensity: " << cornerCloudKeyFrames[i]->points[j].intensity
                          << std::endl;
            }
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }

        if (req.resolution != 0) {
            cout << "\n\nSave resolution: " << req.resolution << endl;

            // down-sample and save corner cloud
            downSizeFilterCorner.setInputCloud(globalCornerCloud);
            downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
            downSizeFilterCorner.filter(*globalCornerCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
            // down-sample and save surf cloud
            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
            downSizeFilterSurf.filter(*globalSurfCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
        } else {
            // save corner cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
            // save surf cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
        }

        // save global point cloud map
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;

        int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
        res.success = ret == 0;

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n" << endl;

        return true;
    }

    void visualizeGlobalMapThread() {
        ros::Rate rate(0.2);
        while (ros::ok()) {
            rate.sleep();
            publishGlobalMap();
        }

        if (savePCD == false) return;

        lio_sam::save_mapRequest req;
        lio_sam::save_mapResponse res;

        if (!saveMapService(req, res)) {
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap() {
        if (pubLaserCloudSurround.getNumSubscribers() == 0) return;

        if (cloudKeyPoses3D->points.empty() == true) {
            // print reason of return
            // ROS_INFO("cloudKeyPoses3D->points is empty");
            return;
        }

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        ;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap,
                                      0);
        mtx.unlock();
        // ROS_INFO("kd-tree set");
        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            for (int i = 0; i < (int)cloudKeyPoses3D->size(); ++i) {
                globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
                globalMapKeyPoses->push_back(cloudKeyPoses3D->points[i]);
            }
        // ROS_INFO("key poses pushed");
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;  // for global map visualization
        *globalMapKeyPosesDS = *globalMapKeyPoses;
        // ROS_INFO("key poses downsampled");
        downSizeFilterGlobalMapKeyPoses.setLeafSize(
            globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
            globalMapVisualizationPoseDensity);  // for global map visualization downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for (auto& pt : globalMapKeyPosesDS->points) {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }
        // ROS_INFO("kd-tree search done");

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius) continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // ROS_INFO("key frames extracted");
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;  // for global map visualization
        *globalMapKeyFramesDS = *globalMapKeyFrames;
        // ROS_INFO("key frames downsampled");
        downSizeFilterGlobalMapKeyFrames.setLeafSize(
            globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
            globalMapVisualizationLeafSize);  // for global map visualization downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
        // ROS_INFO("published cloud");
    }

    void loopClosureThread() {
        if (loopClosureEnableFlag == false) return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok()) {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg) {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2) return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5) loopInfoVec.pop_front();
    }

    void performLoopClosure() {
        if (cloudKeyPoses3D->points.empty() == true) return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0) publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        std::cout << ANSI_COLOR_WHITE << "laserCloudSurfFromMapDS size: " << laserCloudSurfFromMapDS->points.size() << ANSI_COLOR_RESET << std::endl;

        // pcl::shared_ptr<pcl::PointCloud<PointType>> globalKeyFrameCloud(new pcl::PointCloud<PointType>());
        // globalKeyFrameCloud->points.resize(surround_surf->size());
        // for (int i = 0; i < (int)surround_surf->size(); i++) {
        //     globalKeyFrameCloud->points[i].x = surround_surf->points[i].x;
        //     globalKeyFrameCloud->points[i].y = surround_surf->points[i].y;
        //     globalKeyFrameCloud->points[i].z = surround_surf->points[i].z;
        //     globalKeyFrameCloud->points[i].intensity = surround_surf->points[i].intensity;
        // }
        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(laserCloudSurfFromMapDS);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore) return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;  // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int* latestID, int* closestID) {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff) {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int* latestID, int* closestID) {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty()) return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff) return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2) return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i) {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i) {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre) return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum) {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize) continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty()) return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure() {
        if (loopIndexContainer.empty()) return;

        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it) {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    void updateInitialGuess() {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty()) {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization) transformTobeMapped[2] = 0;

            lastImuTransformation =
                pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);  // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX, cloudInfo.initialGuessY, cloudInfo.initialGuessZ,
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false) {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation =
                    pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);  // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true) {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation =
                pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);  // save imu before return;
            return;
        }
    }

    void extractForLoopClosure() {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractForLoopClosureFromMap() {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());

        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby() {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(map.cloudKeyPoses3D_);  // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(map.cloudKeyPoses3D_->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for (auto& pt : surroundingKeyPosesDS->points) {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = map.cloudKeyPoses3D_->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        // *laserCloudCornerFromMap = *_mapCornerCloud;
        // *laserCloudSurfFromMap = *_mapSurfCloud;
        for (int i = 0; i < (int)cloudToExtract->size(); ++i) {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius) {
                // Reason to continue
                // ROS_INFO("Timestamp: %f", timeLaserInfoStamp.toSec());
                // print point distance value and say it is greater than surroundingKeyframeSearchRadius
                // ROS_INFO("Point distance %f is greater than surroundingKeyframeSearchRadius %f",
                //  pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()), surroundingKeyframeSearchRadius);
                continue;
            }

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) {
                // ROS_INFO("Timestamp: %f", timeLaserInfoStamp.toSec());
                // ROS_INFO("Keyframe %d not found in map", thisKeyInd);
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp =
                    *transformPointCloud(map.corner_keyframes_[thisKeyInd], &map.cloudKeyPoses6D_->points[thisKeyInd]);
                // laserCloudCornerTemp->points[0].intensity = thisKeyInd;
                pcl::PointCloud<PointType> laserCloudSurfTemp =
                    *transformPointCloud(map.surf_keyframes_[thisKeyInd], &map.cloudKeyPoses6D_->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                // ROS_INFO("Timestamp: %f", timeLaserInfoStamp.toSec());
                // ROS_INFO("Keyframe %d not found in map", thisKeyInd);
                // pcl::PointCloud<PointType> laserCloudCornerTemp = *_mapCornerCloud;
                // pcl::PointCloud<PointType> laserCloudSurfTemp = *_mapSurfCloud;
                // *laserCloudCornerFromMap = laserCloudCornerTemp;
                // *laserCloudSurfFromMap = laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        std::cout << ANSI_COLOR_GREEN << "laserCloudCornerFromMapDSNum: " << laserCloudCornerFromMapDSNum << ANSI_COLOR_RESET << std::endl;
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
        std::cout << ANSI_COLOR_GREEN << "laserCloudSurfFromMapDSNum: " << laserCloudSurfFromMapDSNum << ANSI_COLOR_RESET << std::endl;
        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000) laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames() {
        if (cloudKeyPoses3D->points.empty() == true) return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan() {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap() { transPointAssociateToMap = trans2Affine3f(transformTobeMapped); }

    void cornerOptimization() {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                    // ROS_INFO("Timestamp: %f", timeLaserInfoStamp.toSec());
                    // ROS_INFO("Corner NKS point %d, %d: %f", i, j, pointSearchSqDis[j]);
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                                      ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                                      ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la =
                        ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                        a012 / l12;

                    float lb =
                        -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                        a012 / l12;

                    float lc =
                        -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                        a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization() {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                    // ROS_INFO("Timestamp: %f", timeLaserInfoStamp.toSec());
                    // ROS_INFO("Surf NKS point %d, %d: %f", i, j, pointSearchSqDis[j]);
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x + pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs() {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i) {
            if (laserCloudOriSurfFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount) {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            std::cout << ANSI_COLOR_YELLOW << "Timestamp: " << timeLaserInfoCur << ", laserCloudSelNum: " << laserCloudSelNum << ANSI_COLOR_RESET
                      << std::endl;
            ROS_WARN("less than 50 points for optimization");
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x +
                        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y +
                        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x +
                        ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x +
                        (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                        ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) + pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) + pow(matX.at<float>(4, 0) * 100, 2) + pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            // ROS_INFO("transformTobeMapped: roll: %f, pitch: %f, yaw: %f, x: %f, y: %f, z: %f", transformTobeMapped[0], transformTobeMapped[1],
            //  transformTobeMapped[2], transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
            //  print ros timestamp
            // ROS_INFO("converged and ros timestamp: %f", timeLaserInfoCur);
            return true;  // converged
        }
        return false;  // keep optimizing
    }

    void scan2MapOptimizationLocalization() {
        if (cloudKeyPoses3D->points.empty()) return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum) {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            std::cout << ANSI_COLOR_BLUE << "Timestamp: " << timeLaserInfoCur << ", Optimizing " << ANSI_COLOR_RESET << std::endl;
            for (int iterCount = 0; iterCount < 30; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true) {
                    std::cout << ANSI_COLOR_BLUE << "Timestamp: " << timeLaserInfoCur << ", LM Optimization Successfull transforming "
                              << ANSI_COLOR_RESET << std::endl;
                    break;
                }
            }

            transformUpdate();
            std::cout << ANSI_COLOR_BLUE << "Timestamp: " << timeLaserInfoCur << ", transformationPose: " << transformTobeMapped[3] << ", "
                      << transformTobeMapped[4] << ", " << transformTobeMapped[5] << ", " << transformTobeMapped[0] << ", " << transformTobeMapped[1]
                      << ", " << transformTobeMapped[2] << ANSI_COLOR_RESET << std::endl;
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate() {
        if (cloudInfo.imuAvailable == true) {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool saveFrame() {
        if (cloudKeyPoses3D->points.empty()) return true;

        if (sensor == SensorType::LIVOX) {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0) return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < surroundingkeyframeAddingAngleThreshold && abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold && sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor() {
        if (cloudKeyPoses3D->points.empty()) {
            noiseModel::Diagonal::shared_ptr priorNoise =
                noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());  // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        } else {
            noiseModel::Diagonal::shared_ptr odometryNoise =
                noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor() {
        if (gpsQueue.empty()) return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0) return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold) return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty()) {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2) {
                // message too old
                gpsQueue.pop_front();
            } else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2) {
                // message too new
                break;
            } else {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold) continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation) {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor() {
        if (loopIndexQueue.empty()) return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor() {
        if (saveFrame() == false) return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true) {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size();  // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points

        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
        for (int i = 0; i < thisCornerKeyFrame->points.size(); i++) {
            thisCornerKeyFrame->points[i].intensity = thisPose3D.intensity;
        }
        for (int i = 0; i < thisSurfKeyFrame->points.size(); i++) {
            thisSurfKeyFrame->points[i].intensity = thisPose3D.intensity;
        }

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses() {
        if (cloudKeyPoses3D->points.empty()) return;

        if (aLoopIsClosed == true) {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i) {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry() {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar =
            tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                          tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental;  // incremental odometry msg
        static Eigen::Affine3f increOdomAffine;          // incremental odometry in affine
        if (lastIncreOdomPubFlag == false) {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true) {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames() {
        if (cloudKeyPoses3D->points.empty()) return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0) {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0) {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size()) {
                lio_sam::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_sam");

    mapOptimizationLocalization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimizationLocalization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimizationLocalization::visualizeGlobalMapThread, &MO);

    ros::spin();

    // loopthread.join();
    visualizeMapThread.join();

    return 0;
}
