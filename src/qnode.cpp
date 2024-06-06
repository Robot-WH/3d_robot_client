/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "qnode.hpp"

#include <ros/network.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <qelapsedtimer.h>
#include <QDebug>
#include <sstream>
#include <string>

namespace ros_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/
////////////////////////////////////////////////////////////////////////////////////////////////////////
QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {
  map_frame = "odom";
  base_frame = "base";
  qRegisterMetaType<sensor_msgs::BatteryState>("sensor_msgs::BatteryState");
  qRegisterMetaType<RobotPose>("RobotPose");
  qRegisterMetaType<RobotStatus>("RobotStatus");
  qRegisterMetaType<QVector<int>>("QVector<int>");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QNode::~QNode() {
//  qDebug() << "~QNode()";
  if (ros::isStarted()) {
//    qDebug() << "shutdown()";
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
//  qDebug() << "wait()";
  wait();   // 等待线程结束
//  qDebug() << "wait() done";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool QNode::init() {
  ros::init(init_argc, init_argv, "ros_qt5_gui_app",
            ros::init_options::AnonymousName);
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  SubAndPubTopic();
  start();
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::init 初始化的函数
/// \param master_url
/// \param host_url
/// \return
///
bool QNode::init(const std::string& master_url, const std::string& host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "ros_qt5_gui_app", ros::init_options::AnonymousName);
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  SubAndPubTopic();
  start();                 // 启动线程     线程执行的是 run()函数
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::SubAndPubTopic 创建订阅者与发布者
///
void QNode::SubAndPubTopic() {
  ros::NodeHandle n;
  // Add your ros communications here.
  QSettings settings("ros_qt5_gui_app", "Displays");
  // 速度控制话题
  //  cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  // 重置设置话题
  reset_pub = n.advertise<std_msgs::Bool>("cmd_reset", 1);
  getWorkSpace_pub = n.advertise<std_msgs::Bool>("getWorkSpace", 1);

  // 创建速度话题的订阅者
  //  cmdVel_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.toStdString(), 200,
  //                                               &QNode::speedCallback, this);
  //  battery_sub = n.subscribe(batteryState_topic.toStdString(), 1000,
  //                            &QNode::batteryCallback, this);
  // 地图订阅
  QString occ_grid_topic = settings.value("Map/occ_grid_topic", QString("/map")).toString();
  map_sub = n.subscribe(occ_grid_topic.toStdString(), 1000, &QNode::gridmapCallback, this);
  roboOdomPos_sub_ = n.subscribe("/fusion_odom", 10, &QNode::roboOdomPosCallback, this);
  odom_to_map_sub_ = n.subscribe("/odom_to_map", 10, &QNode::odomToMapCallback, this);
  // 激光雷达点云话题订阅
  stable_laser_point_topic = settings.value("Laser/stable_laser_point", QString("/stable_laser_points")).toString();
  stable_laser_point_sub_ = n.subscribe(stable_laser_point_topic.toStdString(), 1000,
                           &QNode::stableLaserPointCallback, this);
  //  qDebug() << "stable_laser_point_topic: " << stable_laser_point_topic;

  dynamic_laser_point_topic = settings.value("Laser/dynamic_laser_point_topic", QString("/dynamic_laser_points")).toString();
  dynamic_laser_point_sub_ = n.subscribe(dynamic_laser_point_topic.toStdString(), 1000,
                           &QNode::dynamicLaserPointCallback, this);
  global_lidar_map_sub_ = n.subscribe("/global_map", 10,
                                      &QNode::globalLidarMapCallback, this);
  lidar_stablePoint_sub_ = n.subscribe("/preprocessed_pointcloud", 10,
                                      &QNode::lidarStablePointCallback, this);
//  qDebug() << "stable_laser_point_topic: " << stable_laser_point_topic;
  // 图像话题的回调
  m_compressedImgSub0_ = n.subscribe("front_camera", 10,
                                     &QNode::imageCallback0, this);
//  image_transport::ImageTransport it(n);
//  m_imageMapPub = it.advertise("image/map", 10);

//  //导航目标点发送话题
//  goal_pub = n.advertise<geometry_msgs::PoseStamped>(
//      naviGoal_topic.toStdString(), 1000);
//  //全局规划Path
//  m_plannerPathSub =
//      n.subscribe(path_topic, 1000, &QNode::plannerPathCallback, this);
//  m_initialposePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
//      initPose_topic.toStdString(), 10);

  workspace_sub = n.subscribe("/workspace", 100, &QNode::workspaceCallback, this);

  m_robotPoselistener = new tf::TransformListener;
  m_Laserlistener = new tf::TransformListener;
//  try {
//    m_robotPoselistener->waitForTransform(map_frame, base_frame, ros::Time(0),
//                                          ros::Duration(0.4));
//    m_Laserlistener->waitForTransform(map_frame, laser_frame, ros::Time(0),
//                                      ros::Duration(0.4));
//  } catch (tf::TransformException& ex) {
//    log(Error, ("laser and robot pose tf listener: " + QString(ex.what()))
//                   .toStdString());
//  }
//  movebase_client = new MoveBaseClient("move_base", true);
//  movebase_client->waitForServer(ros::Duration(1.0));
  set_space_client = n.serviceClient<lifelong_backend::SetSpace>("SetSpace");
  save_traj_client = n.serviceClient<lifelong_backend::SaveTraj>("SaveTraj");
  set_traj_client = n.serviceClient<lifelong_backend::SetTraj>("SetTraj");
  set_workMode_client = n.serviceClient<lifelong_backend::SetCommand>("SetWorkMode");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QMap<QString, QString> QNode::get_topic_list() {
  ros::master::V_TopicInfo topic_list;
  ros::master::getTopics(topic_list);
  QMap<QString, QString> res;
  for (auto topic : topic_list) {
    res.insert(QString::fromStdString(topic.name),
               QString::fromStdString(topic.datatype));
  }
  return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::plannerPathCallback planner的路径话题回调
/// \param path
///
void QNode::plannerPathCallback(nav_msgs::Path::ConstPtr path) {
  plannerPoints.clear();
  for (int i = 0; i < (int)path->poses.size(); i++) {
    QPointF roboPos = transWordPoint2Scene(QPointF(
        path->poses[i].pose.position.x, path->poses[i].pose.position.y));
    plannerPoints.append(roboPos);
  }
  emit plannerPath(plannerPoints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::stableLaserPointCallback 激光雷达静态点云话题回调
/// \param laser_msg
///
void QNode::stableLaserPointCallback(sensor_msgs::PointCloudConstPtr laser_msg) {
  stableLaserPoints.clear();

  for (int i = 0; i < (int)laser_msg->points.size(); i++) {
    QPointF pos(laser_msg->points[i].x, laser_msg->points[i].y);
    stableLaserPoints.append(pos);
  }
  emit updateStableLaserScan(stableLaserPoints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::dynamicLaserPointCallback 激光雷达动态点云话题回调
/// \param laser_msg
///
void QNode::dynamicLaserPointCallback(sensor_msgs::PointCloudConstPtr laser_msg) {
  dynamicLaserPoints.clear();

  for (int i = 0; i < (int)laser_msg->points.size(); i++) {
    QPointF pos(laser_msg->points[i].x, laser_msg->points[i].y);
    dynamicLaserPoints.append(pos);
  }
  emit updateDynamicLaserScan(dynamicLaserPoints);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::dynamicLaserPointCallback 激光雷达动态点云话题回调
/// \param laser_msg
///
void QNode::globalLidarMapCallback(sensor_msgs::PointCloud2ConstPtr map) {
  // qDebug() << "globalLidarMapCallback";
  openGLWidget_->SetGlobalLidarMap(map);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::lidarStablePointCallback(sensor_msgs::PointCloud2ConstPtr scan) {
  qDebug() << "lidarStablePointCallback";
  openGLWidget_->SetLidarStablePoint(scan);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::roboOdomPosCallback(nav_msgs::Odometry msg) {
  qDebug() << "roboOdomPosCallback";
  // 提取四元数和位置
  geometry_msgs::Quaternion q = msg.pose.pose.orientation;
  geometry_msgs::Point p = msg.pose.pose.position;
  qDebug() << "p x:" << p.x << ",y: " << p.y << ",z:" << p.z;
  // 将ROS四元数转换为Qt四元数
  QQuaternion qtQ(q.w, q.x, q.y, q.z);
  // 创建平移矩阵（基于位置）
  QMatrix4x4 translationMatrix;
  translationMatrix.translate(p.x, p.y, p.z);
  translationMatrix.rotate(qtQ);
  openGLWidget_->SetRoboPose(translationMatrix);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::odomToMapCallback(nav_msgs::Odometry msg) {
  qDebug() << "odomToMapCallback";
  // 提取四元数和位置
  geometry_msgs::Quaternion q = msg.pose.pose.orientation;
  geometry_msgs::Point p = msg.pose.pose.position;
  // qDebug() << "p x:" << p.x << ",y: " << p.y << ",z:" << p.z;
  // 将ROS四元数转换为Qt四元数
  QQuaternion qtQ(q.w, q.x, q.y, q.z);
  // 创建平移矩阵（基于位置）
  QMatrix4x4 translationMatrix;
  translationMatrix.translate(p.x, p.y, p.z);
  translationMatrix.rotate(qtQ);
  openGLWidget_->SetOdomToMapTrans(translationMatrix);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::updateRobotPose() {
  try {
    tf::StampedTransform transform;
    m_robotPoselistener->lookupTransform(map_frame, base_frame, ros::Time(0),
                                         transform);
    tf::Quaternion q = transform.getRotation();
    double x = transform.getOrigin().getX();
    double y = transform.getOrigin().getY();
    tf::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
//    qDebug() << "robot x: " << x << ", y: " << y << ", yaw: " << yaw;
//    QPointF roboPos = transWordPoint2Scene(QPointF(x, y));
    RobotPose pos{x, y, yaw};
    emit updateRoboPose(pos);
  } catch (tf::TransformException& ex) {
//    log(Error,
//        ("robot pose tf transform: " + QString(ex.what())).toStdString());
//    try {
//      m_robotPoselistener->waitForTransform(map_frame, base_frame, ros::Time(0),
//                                            ros::Duration(0.4));
//      m_Laserlistener->waitForTransform(map_frame, laser_frame, ros::Time(0),
//                                        ros::Duration(0.4));
//    } catch (tf::TransformException& ex) {
//      log(Error,
//          ("robot pose tf transform: " + QString(ex.what())).toStdString());
//    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::batteryCallback(const sensor_msgs::BatteryState& message) {
  emit batteryState(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::myCallback(const std_msgs::Float64& message_holder) {
  qDebug() << message_holder.data << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::set_goal 发布导航目标点信息
/// \param frame
/// \param x
/// \param y
/// \param z
/// \param w
///
void QNode::set_goal(QString frame, double x, double y, double z, double w) {
  geometry_msgs::PoseStamped goal;
  //设置frame
  goal.header.frame_id = frame.toStdString();
  //设置时刻
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0;
  goal.pose.orientation.z = z;
  goal.pose.orientation.w = w;
  goal_pub.publish(goal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::SetReset() {
  std_msgs::Bool flag;
  flag.data = true;
  reset_pub.publish(flag);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::SetGridMapShowFlag(bool flag) {
  gridmap_show_flag = flag;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::SetOpenGLWidget(PointCloudOpenGLWidget *openGLWidget) {
  openGLWidget_ = openGLWidget;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::gridmapCallback 地图信息订阅回调函数
/// \param msg
///
void QNode::gridmapCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
  static bool first_flag = true;

  if (!gridmap_show_flag) {
    if (!first_flag) {
      return;
    }
  }

  // qDebug() << "gridmapCallback";
  // QElapsedTimer mstimer;
  // mstimer.start();
  int width = msg->info.width;
  int height = msg->info.height;
//  int width = 100;
//  int height = 100;
  QImage map_image(width, height, QImage::Format_RGB32);

  for (int i = 0; i < (int)msg->data.size(); i++) {
    int x = i % width;
    int y = (int)i / width;
    //计算像素值
    QColor color;
    if (msg->data[i] == 100) {
      color = Qt::black;  // black
    } else if (msg->data[i] == 0) {
      color = Qt::white;  // white
    } else if (msg->data[i] == -1) {
      color = Qt::gray;  // gray
    }
    // 在ROS中，y朝上，而在Qt中，y朝下
    map_image.setPixel(x, height - y - 1, qRgb(color.red(), color.green(), color.blue()));
  }
//  for (int i = 0; i < 10000; i++) {
//    int x = i % width;
//    int y = (int)i / width;
//    //计算像素值
//    QColor color;
//     color = Qt::red;  // black
//    // 在ROS中，y朝上，而在Qt中，y朝下，因此这里刚好把图片给颠倒了
//    map_image.setPixel(x, y, qRgb(color.red(), color.green(), color.blue()));
//  }
  //延y翻转地图 因为解析到的栅格地图的坐标系原点为左下角
  //但是图元坐标系为左上角度
//  map_image = rotateMapWithY(map_image);
  //计算翻转后的图元坐标系原点的世界坐标
//  double origin_x_ = origin_x;
//  double origin_y_ = origin_y + height * m_mapResolution;
  //地图坐标系原点在图元坐标系下的坐标

//  m_wordOrigin.setX(origin_x / m_mapResolution);
//  m_wordOrigin.setY(origin_y / m_mapResolution);
  QPointF mapOrigin;
  mapOrigin.setX(msg->info.origin.position.x);
  mapOrigin.setY(msg->info.origin.position.y);
  m_mapResolution = msg->info.resolution;

  emit updateSubGridMap(map_image, mapOrigin, m_mapResolution, width, height);

  // float time =(double)mstimer.nsecsElapsed()/(double)1000000;
  // qDebug() <<"gridmapCallback time= " <<time<<"ms";// 输出运行时间（ms）
  first_flag = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::speedCallback 速度回调函数
/// \param msg
///
void QNode::speedCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  emit speed_x(msg->twist.twist.linear.x);
  emit speed_y(msg->twist.twist.linear.y);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::run() {
  ros::Rate loop_rate(m_frameRate);
  ros::AsyncSpinner spinner(m_threadNum);
  spinner.start();
  //当当前节点没有关闭时
  while (ros::ok()) {
    updateRobotPose();
//    emit updateRobotStatus(RobotStatus::normal);
    loop_rate.sleep();    // 没有ros 消息的话会一直阻塞
  }
  //如果当前节点关闭
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief QNode::move_base 发布机器人速度控制
/// \param k
/// \param speed_linear
/// \param speed_trun
///
void QNode::move_base(char k, float speed_linear, float speed_trun) {
  std::map<char, std::vector<float>> moveBindings {
      {'i', {1, 0, 0, 0}},  {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
      {'l', {0, 0, 0, -1}}, {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
      {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}}, {'O', {1, -1, 0, 0}},
      {'I', {1, 0, 0, 0}},  {'J', {0, 1, 0, 0}},   {'L', {0, -1, 0, 0}},
      {'U', {1, 1, 0, 0}},  {'<', {-1, 0, 0, 0}},  {'>', {-1, -1, 0, 0}},
      {'M', {-1, 1, 0, 0}}, {'t', {0, 0, 1, 0}},   {'b', {0, 0, -1, 0}},
      {'k', {0, 0, 0, 0}},  {'K', {0, 0, 0, 0}}};
  char key = k;
  //计算是往哪个方向
  float x = moveBindings[key][0];
  float y = moveBindings[key][1];
  float z = moveBindings[key][2];
  float th = moveBindings[key][3];
  //计算线速度和角速度
  float speed = speed_linear;
  float turn = speed_trun;
  // Update the Twist message
  geometry_msgs::Twist twist;
  twist.linear.x = x * speed;
  twist.linear.y = y * speed;
  twist.linear.z = z * speed;

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = th * turn;

  // Publish it and resolve any remaining callbacks
  cmd_pub.publish(twist);
  ros::spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// 订阅图片话题，并在label上显示
// void QNode::SubImage(QString topic, int frame_id) {
//   ros::NodeHandle n;
// //  image_transport::ImageTransport it_(n);
//   if (frame_id == 0) {
//     m_compressedImgSub0 =
//         n.subscribe(topic.toStdString(), 100, &QNode::imageCallback0, this);
//   } else if (frame_id == 1) {
//     m_compressedImgSub1 =
//         n.subscribe(topic.toStdString(), 100, &QNode::imageCallback1, this);
//   }
//   ros::spinOnce();
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::pubGetWorkSpaceCmd() {
  std_msgs::Bool flag;
  flag.data = true;
  getWorkSpace_pub.publish(flag);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::pub2DPose(QPointF start_pose,QPointF end_pose){
    start_pose =transScenePoint2Word(start_pose);
    end_pose =transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y()-start_pose.y(),end_pose.x()-start_pose.x());
    geometry_msgs::PoseWithCovarianceStamped goal;
    //设置frame
    goal.header.frame_id = "map";
    //设置时刻
    goal.header.stamp = ros::Time::now();
    goal.pose.pose.position.x = start_pose.x();
    goal.pose.pose.position.y = start_pose.y();
    goal.pose.pose.position.z = 0;
    goal.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
    m_initialposePub.publish(goal);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::pub2DGoal(QPointF start_pose,QPointF end_pose){
    start_pose =transScenePoint2Word(start_pose);
    end_pose =transScenePoint2Word(end_pose);
    double angle = atan2(end_pose.y()-start_pose.y(),end_pose.x()-start_pose.x());
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = start_pose.x();
    goal.target_pose.pose.position.y = start_pose.y();
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, angle);
    movebase_client->sendGoal(goal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QPointF QNode::transScenePoint2Word(QPointF pose) {
  QPointF res;
  res.setX((pose.x() - m_wordOrigin.x()) * m_mapResolution);
  // y坐标系相反
  res.setY(-1 * (pose.y() - m_wordOrigin.y()) * m_mapResolution);
  return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QPointF QNode::transWordPoint2Scene(QPointF pose) {
  //    qDebug()<<pose;
  QPointF res;
  res.setX(m_wordOrigin.x() + pose.x() / m_mapResolution);
  res.setY(m_wordOrigin.y() - (pose.y() / m_mapResolution));
  return res;
}

//void QNode::pub_imageMap(QImage map) {
//  cv::Mat image = QImage2Mat(map);
//  sensor_msgs::ImagePtr msg =
//      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//  m_imageMapPub.publish(msg);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// 图像话题的回调函数
 void QNode::imageCallback0(const sensor_msgs::ImageConstPtr& msg) {
   try {
     // 深拷贝转换为opencv类型
     cv_bridge::CvImagePtr cv_ptr_compressed =
         cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cv_ptr_compressed->image.empty()) {
      qDebug() << "image.empty()";
    } else {
      // cv::imshow("USB Camera", cv_ptr_compressed->image);  // 显示图像
      // char key = cv::waitKey(10);
    }
    
    QImage im = Mat2QImage(cv_ptr_compressed->image);
     emit showImage(0, im);
    //  qDebug() << "imageCallback0";
   } catch (cv_bridge::Exception& e) {
     log(Error, ("video frame0 exception: " + QString(e.what())).toStdString());
     return;
   }
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 void QNode::workspaceCallback(const ros_msg::workspace_info& msg) {
   std::cout << "workspaceCallback" << std::endl;
   // std::cout << "name: " << name << std::endl;
   for (int i = 0; i < msg.space_name.size(); ++i) {
     workspace.push_back(msg.space_name[i]);
   }
   workspace_update = true;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 bool QNode::isWorkspaceUpdate() {
   return workspace_update;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 const std::vector<std::string>& QNode::getWorkspace() {
   workspace_update = false;
   return workspace;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
// void QNode::imageCallback1(const sensor_msgs::CompressedImageConstPtr& msg) {
//   cv_bridge::CvImagePtr cv_ptr;

//   try {
//     //深拷贝转换为opencv类型
//     cv_bridge::CvImagePtr cv_ptr_compressed =
//         cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     QImage im = Mat2QImage(cv_ptr_compressed->image);
//     emit Show_image(1, im);
//   } catch (cv_bridge::Exception& e) {
//     log(Error, ("video frame0 exception: " + QString(e.what())).toStdString());
//     return;
//   }
// }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 void QNode::setSpaceCall(const QString& space_name, std::vector<uint16_t>& traj_id) {
   lifelong_backend::SetSpaceRequest req;
   req.space_name = space_name.toStdString();
   lifelong_backend::SetSpaceResponse res;
   set_space_client.call(req, res);
   qDebug() << "setSpaceCall space_name:" << space_name << "res: " << res.traj_id;
   traj_id = res.traj_id;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 void QNode::saveTrajCall(const QString& traj_name, std::vector<uint16_t>& traj_id) {
   lifelong_backend::SaveTrajRequest req;
   req.traj_name = traj_name.toStdString();
   lifelong_backend::SaveTrajResponse res;
   save_traj_client.call(req, res);
   traj_id = res.traj_id;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 void QNode::setTrajCall(const uint16_t& traj_id, uint8_t& success) {
   lifelong_backend::SetTrajRequest req;
   req.traj_id = traj_id;
   lifelong_backend::SetTrajResponse res;
   set_traj_client.call(req, res);
   qDebug() << "setTrajCall traj_id:" << traj_id << "res: " << (int)res.success;
   success = res.success;
 }

 ////////////////////////////////////////////////////////////////////////////////////////////////////////
 void QNode::setWorkModeCall(const uint16_t& cmd, uint8_t& success) {
   lifelong_backend::SetCommandRequest req;
   req.type = 1;
   req.cmd = cmd;
   lifelong_backend::SetCommandResponse res;
   set_workMode_client.call(req, res);
   qDebug() << "res: " << (int)res.success;
   success = res.success;
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////
QImage QNode::rotateMapWithY(QImage map) {
  QImage res = map;
  for (int x = 0; x < map.width(); x++) {
    for (int y = 0; y < map.height(); y++) {
      res.setPixelColor(x, map.height() - y - 1, map.pixel(x, y));
    }
  }
  return res;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QImage QNode::Mat2QImage(cv::Mat const& src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//cv::Mat QNode::QImage2Mat(QImage& image) {
//  cv::Mat mat;
//  switch (image.format()) {
//    case QImage::Format_ARGB32:
//    case QImage::Format_RGB32:
//    case QImage::Format_ARGB32_Premultiplied:
//      mat = cv::Mat(image.height(), image.width(), CV_8UC4,
//                    (void*)image.constBits(), image.bytesPerLine());
//      break;
//    case QImage::Format_RGB888:
//      mat = cv::Mat(image.height(), image.width(), CV_8UC3,
//                    (void*)image.constBits(), image.bytesPerLine());
//      cv::cvtColor(mat, mat, CV_BGR2RGB);
//      break;
//    case QImage::Format_Indexed8:
//      mat = cv::Mat(image.height(), image.width(), CV_8UC1,
//                    (void*)image.constBits(), image.bytesPerLine());
//      break;
//  }
//  return mat;
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void QNode::log(const LogLevel& level, const std::string& msg) {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info): {
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn): {
      emit updateRobotStatus(RobotStatus::warn);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error): {
      emit updateRobotStatus(RobotStatus::error);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal): {
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

}  // namespace ros_qt5_gui_app
