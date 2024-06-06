/**
 * @file /include/ros_qt5_gui_app/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_qt5_gui_app_QNODE_HPP_
#define ros_qt5_gui_app_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <cv_bridge/cv_bridge.h>  //cv_bridge
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
//#include <image_transport/image_transport.h>  //image_transport
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>  //图像编码格式

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <QDebug>
#include <QImage>
#include <QLabel>
#include <QSettings>
#include <QStringListModel>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <map>
#include <string>

#include "roboItem.h"
#include "ros_msg/control_info.h"
#include "ros_msg/workspace_info.h"
#include "ros_service/SetSpace.h"
#include "ros_service/SaveTraj.h"
#include "ros_service/SetTraj.h"
#include "ros_service/SetCommand.h"
#include "PointCloudOpenGLWidget.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_qt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void move_base(char k, float speed_linear, float speed_trun);
  void set_goal(QString frame, double x, double y, double z, double w);
  void SetReset();
  void SetGridMapShowFlag(bool flag);
  // void SubImage(QString topic, int frame_id);
//  void pub_imageMap(QImage map);
  void pubGetWorkSpaceCmd();
  void workspaceCallback(const ros_msg::workspace_info& msg);
  void setSpaceCall(const QString& space_name, std::vector<uint16_t>& traj_cnt);
  void saveTrajCall(const QString& traj_name, std::vector<uint16_t>& traj_id);
  void setTrajCall(const uint16_t& traj_id, uint8_t& success);
  void setWorkModeCall(const uint16_t& cmd, uint8_t& success);
  void SetOpenGLWidget(PointCloudOpenGLWidget *openGLWidget);
  QPointF transScenePoint2Word(QPointF pos);
  QPointF transWordPoint2Scene(QPointF pos);
  QMap<QString, QString> get_topic_list();
  int mapWidth{0};
  int mapHeight{0};
  void run();             // 线程执行函数    继承自QThread
  bool isWorkspaceUpdate();
  const std::vector<std::string>& getWorkspace();

  /*********************
  ** Logging
  **********************/
  enum LogLevel { Debug, Info, Warn, Error, Fatal };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

 public slots:
  void pub2DPose(QPointF,QPointF);
  void pub2DGoal(QPointF,QPointF);

 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void speed_x(double x);
  void speed_y(double y);
  void batteryState(sensor_msgs::BatteryState);
  void Master_shutdown();
  void showImage(int, QImage);
  void updateRoboPose(RobotPose pos);
  void updateMap(QImage map);
  void updateSubGridMap(QImage map, QPointF mapOrigin, float res, int width, int height);
  void plannerPath(QPolygonF path);
  void updateStableLaserScan(QPolygonF points);
  void updateDynamicLaserScan(QPolygonF points);
  void updateRobotStatus(RobotStatus status);

 private:
  int init_argc;
  char **init_argv;
  ros::Publisher chatter_publisher;
  ros::Subscriber cmdVel_sub;
  ros::Subscriber chatter_subscriber;
  ros::Subscriber pos_sub;
  ros::Subscriber stable_laser_point_sub_;
  ros::Subscriber dynamic_laser_point_sub_;
  ros::Subscriber battery_sub;
  ros::Subscriber m_plannerPathSub;
  ros::Subscriber m_compressedImgSub0_;
  ros::Subscriber m_compressedImgSub1;
  ros::Subscriber global_lidar_map_sub_;
  // 信息交互
  ros::Subscriber workspace_sub;

  ros::Publisher goal_pub;
  ros::Publisher cmd_pub;
  ros::Publisher reset_pub;
  ros::Publisher m_initialposePub;
  ros::Publisher getWorkSpace_pub;
  ros::ServiceClient set_space_client;
  ros::ServiceClient save_traj_client;
  ros::ServiceClient set_traj_client;
  ros::ServiceClient set_workMode_client;
//  image_transport::Publisher m_imageMapPub;
  MoveBaseClient *movebase_client;
  QStringListModel logging_model;
  QString show_mode = "control";
  //图像订阅
//  image_transport::Subscriber image_sub0;
  //地图订阅
  ros::Subscriber map_sub;
  //图像format
  QString video0_format;
  QString odom_topic;
  QString batteryState_topic;
  QString pose_topic;
  QString stable_laser_point_topic;
  QString dynamic_laser_point_topic;
  QString map_topic;
  QString initPose_topic;
  QString naviGoal_topic;
  std::string path_topic;
  QPolygon mapPonits;
  QPolygonF plannerPoints;
  QPolygonF stableLaserPoints;
  QPolygonF dynamicLaserPoints;
  int m_threadNum = 2;
  int m_frameRate = 40;
  //地图 0 0点坐标对应世界坐标系的坐标
  float m_mapOriginX;
  float m_mapOriginY;
  //世界坐标系原点在图元坐标系坐标
  QPointF m_wordOrigin;
  //地图一个像素对应真实世界的距离
  float m_mapResolution;
  //地图是否被初始化
  bool m_bMapIsInit = false;
  // gird map是否显示
  bool gridmap_show_flag = false;
  bool workspace_update = false;
  // tf::TransformListener m_tfListener(ros::Duration(10));
  // ros::Timer m_rosTimer;
  QImage Mat2QImage(cv::Mat const &src);
//  cv::Mat QImage2Mat(QImage &image);
  QImage rotateMapWithY(QImage map);
  tf::TransformListener *m_robotPoselistener;
  tf::TransformListener *m_Laserlistener;
  std::string base_frame, laser_frame, map_frame;
  std::vector<std::string> workspace;
  PointCloudOpenGLWidget *openGLWidget_;

 private:
  void speedCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void batteryCallback(const sensor_msgs::BatteryState &message);
  void imageCallback0(const sensor_msgs::ImageConstPtr &msg);
  // void imageCallback1(const sensor_msgs::CompressedImageConstPtr &msg);
  void myCallback(const std_msgs::Float64 &message_holder);
  void gridmapCallback(nav_msgs::OccupancyGrid::ConstPtr map);
//  void stableLaserPointCallback(sensor_msgs::LaserScanConstPtr scan);
//  void dynamicLaserPointCallback(sensor_msgs::LaserScanConstPtr scan);
  void stableLaserPointCallback(sensor_msgs::PointCloudConstPtr laser_msg);
  void dynamicLaserPointCallback(sensor_msgs::PointCloudConstPtr laser_msg);
  void globalLidarMapCallback(sensor_msgs::PointCloud2ConstPtr map);
  void plannerPathCallback(nav_msgs::Path::ConstPtr path);
  void SubAndPubTopic();
  void updateRobotPose();
};

}  // namespace ros_qt5_gui_app

#endif /* ros_qt5_gui_app_QNODE_HPP_ */
