#include <QMovie>
#include <QMessageBox>
#include <QDebug>
#include <QNetworkInterface>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "MainWindow.h"
#include "ui_MainWindow.h"    // build/my_gui下 

////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QWidget(parent), ui(new Ui::MainWindow), qt_ros_node_(argc, argv) {
  initUI();    // 初始化UI
  initParm(); // 初始化参数
  // 系统信息输出
  ui->msg_output->setReadOnly(true);
  // 设置最大显示文本容量为100个文本块
  ui->msg_output->document()->setMaximumBlockCount(100);

  frontend_process_ = new QProcess;
  laser_process_ = new QProcess;
  hardware_process_ = new QProcess;
  // 将ROS节点的输出重定向到QTextBrowser控件中
  QObject::connect(frontend_process_, &QProcess::readyReadStandardOutput, [&]() {
      QString output = frontend_process_->readAllStandardOutput();
      ui->msg_output->append(output);
  });
  // 设置QProcess的标准输出通道为MergedChannels
  frontend_process_->setProcessChannelMode(QProcess::MergedChannels);
  //
  bringUpQtRosNode();
  // 信号，槽连接
  connection();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {
  qDebug() << "~MainWindow()";
  delete ui;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::initUI() {
  ui->setupUi(this);
  //  Qt::Window: 指示窗口是一个普通窗口，拥有标题栏和边框。
  //  Qt::WindowTitleHint: 显示窗口标题栏。
  //  Qt::CustomizeWindowHint: 禁用默认的窗口装饰，即关闭、最大化和最小化按钮
  //  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
//  this->setWindowFlags(Qt::FramelessWindowHint);  //去掉标题栏

//  this->setFixedSize(700, 500);
//  // 显示界面的动态背景
////  QMovie* movie = new QMovie("/home/lwh/图片/8059.gif_wh300.gif");
////  ui->label_vedio->setMovie(movie);
//  ui->label_vedio->lower();   // 作为背景
//  ui->label_vedio->setFixedSize(700, 500);
//  ui->label_vedio->setPixmap(QPixmap("/home/lwh/图片/back.jpg"));  // 设置背景图片
////  this->setCentralWidget(backgroundLabel);  // 将QLabel设置为窗口的中心部件
//  ui->label_vedio->setScaledContents(true);  // 设置QLabel的内容自适应缩放
////  movie->setScaledSize(ui->label_vedio->size());  // 将QMovie的缩放大小设置为QLabel的大小
////  movie->start();
////  ui->label_vedio->show();

//  QPalette palette = ui->label_head->palette();  // 获取QLabel的调色板
//  palette.setColor(QPalette::WindowText, Qt::darkMagenta);  // 设置文字颜色为红色
//  ui->label_head->setPalette(palette);  // 应用新的调色板

//  // 标题处的机器人图片
//  ui->label_robot_img->setPixmap(QPixmap("://images/robot.png"));
//  ui->label_robot_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
  ui->pushButton_plus->setIcon(QIcon(":/images/plus.png"));
  ui->pushButton_plus->setFixedHeight(50);
  ui->pushButton_plus->setFixedWidth(50);
  ui->pushButton_minus->setIcon(QIcon(":/images/minus.png"));
  ui->pushButton_minus->setFixedHeight(50);
  ui->pushButton_minus->setFixedWidth(50);
  ui->pushButton_return->setIcon(QIcon(":/images/set_return.png"));
  ui->pushButton_return->setFixedHeight(50);
  ui->pushButton_return->setFixedWidth(50);
  //视图场景加载
  qgraphicsScene_ =
      new QGraphicsScene;  //要用QGraphicsView就必须要有QGraphicsScene搭配着用
  qgraphicsScene_->clear();
  //创建item
  roboItem_ = new ros_qt::roboItem();
  //视图添加item
  qgraphicsScene_->addItem(roboItem_);
  // widget添加视图
  ui->mapViz->setScene(qgraphicsScene_);
//  QRectF visibleRect = ui->mapViz->mapToScene(ui->mapViz->viewport()->geometry()).boundingRect();
//  QPointF visibleRect = ui->mapViz->mapToScene(0, 0);
//  QPointF sceneCenter = ui->mapViz->mapToScene(ui->mapViz->viewport()->rect().topLeft());
//  qDebug() << "sceneCenter w:" << sceneCenter.x() << ", h: " << sceneCenter.y();
  // 视图内   显示内容UI设置
  QString styleSheet = "QCheckBox::indicator{width: 22px;height: 22px;color:rgb(0, 191, 255)}\
        QCheckBox{font-size: 18px;color: rgb(0, 191, 255);}\
        QCheckBox::checked{color:rgb(50,205,50);}\
        QCheckBox::unchecked{color:rgb(119, 136, 153);}\
        ";
  ui->localgridmap_checkBox->setStyleSheet(styleSheet);
  ui->checkBox_2->setStyleSheet(styleSheet);
  ui->checkBox_3->setStyleSheet(styleSheet);
  ui->checkBox_4->setStyleSheet(styleSheet);
  ui->checkBox_5->setStyleSheet(styleSheet);
  ui->checkBox_6->setStyleSheet(styleSheet);
  // 视图内   视角选择UI设置
  styleSheet = "QRadioButton::indicator{width: 22px;height: 22px;color:rgb(0, 191, 255)}\
      QRadioButton{font-size: 18px;color: rgb(0, 191, 255);}\
      QRadioButton::checked{color:rgb(50,205,50);}\
      QRadioButton::unchecked{color:rgb(119, 136, 153);}\
      ";
  ui->radioButton->setStyleSheet(styleSheet);
  ui->radioButton_2->setStyleSheet(styleSheet);
  ui->radioButton_4->setStyleSheet(styleSheet);
  ui->radioButton_4->setChecked(true);

  ui->groupBox_3->setFixedWidth(200);
  ui->groupBox_6->setFixedHeight(250);
  // 放大缩小设置
//  ui->groupBox_5->setFixedWidth(40);
////  ui->groupBox_5->setStyleSheet("QGroupBox { padding: 2px; }");
//  ui->groupBox_5->setStyleSheet("QGroupBox { border: none; }");

  ui->server_connect_Button->setFixedHeight(60);
  ui->server_connect_Button->setFixedWidth(60);
//  // 退出
//  ui->pushButton->setFixedHeight(50);
//  ui->pushButton->setFixedWidth(100);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::initParm() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::connection() {
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::rosShutdown, this,
                 &MainWindow::slot_rosShutdown);
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::updateStableLaserScan, roboItem_,
          &ros_qt::roboItem::paintStableLaserScan);
  QObject::connect(&qt_ros_node_, &ros_qt::QNode::updateDynamicLaserScan, roboItem_,
          &ros_qt::roboItem::paintDynamicLaserScan);
  connect(&qt_ros_node_, &ros_qt::QNode::updateSubGridMap, roboItem_,
          &ros_qt::roboItem::paintSubGridMap);
  // 机器人pose -> map
  connect(&qt_ros_node_, &ros_qt::QNode::updateRoboPose, roboItem_,
          &ros_qt::roboItem::paintRoboPos);
  // 间断跟踪模式下对robopos的处理
  connect(roboItem_, &ros_qt::roboItem::roboPos , this,
          &MainWindow::slot_roboPos);
  // radioButton
  // QRadioButton 有一个状态切换的信号 toggled，即该信号在状态切换时发送；
  connect(ui->radioButton, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {
        roboItem_->SetVisualMode(ros_qt::roboItem::VisualMode::translate_tracking);
      }
  });
  connect(ui->radioButton_2, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {}
  });
  // 设置激光雷达颠倒
  connect(ui->radioButton_3, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {
        roboItem_->SetLaserInverted(true);
      } else {
        roboItem_->SetLaserInverted(false);
      }
  });
  connect(ui->radioButton_4, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {
        roboItem_->SetVisualMode(ros_qt::roboItem::VisualMode::internal_tracking);
      }
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::bringUpQtRosNode() {
  // 读取本机的IP
  QString hostUrl = getLocalIpAddress();
  QString masterUrl = "http://" + hostUrl + ":11311";
  //
  if (!qt_ros_node_.init(masterUrl.toStdString(), hostUrl.toStdString())) {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::red);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS 连接失败");
    ui->label_status_img->setPixmap(QPixmap("://images/false.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
    ui->bringup_button->setEnabled(false);
    return;
  } else {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::darkGreen);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS 连接成功");
    ui->label_status_img->setPixmap(QPixmap("://images/ok.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
//    //初始化视频订阅的显示
//    initVideos();
//    //显示话题列表
//    initTopicList();
//    initOthers();
  }
//  ReadSettings();
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_bringup_button_clicked() {
  // startDetached 启动的进程不能进行通信，也不能输出重定向，但是，主线程退出后不会影响该子进程的执行
  // frontend_process_->startDetached("roslaunch", QStringList() << "calib_fusion_2d" << "frontend.launch");
  //   启动roslaunch命令
// hardware_process_->start("roslaunch", QStringList() << "robot_control" << "robot_control.launch");
// laser_process_->start("roslaunch", QStringList() << "ydlidar_ros_driver" << "lidar.launch");
// // 延时1s  不然启动有问题
// QTime t;
// t.start();
// while(t.elapsed()<1000)//1000ms = 1s
//       QCoreApplication::processEvents();

//frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "frontend.launch");
//  frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "frontend_view.launch");
   frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "dataset_frontend_view.launch");
//  frontend_process_->start("roslaunch", QStringList() << "calib_fusion_2d" << "dataset_frontend.launch");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
QString MainWindow::getLocalIpAddress() {
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
    // 遍历所有接口的IP地址
    for (const QHostAddress &ipAddress : ipAddressesList) {
        // 只获取IPv4地址，忽略IPv6地址和回环地址
        if (ipAddress != QHostAddress::LocalHost && ipAddress.toIPv4Address()) {
            return ipAddress.toString();
        }
    }
    // 如果没有找到有效的IP地址，则返回空字符串
    return QString();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_server_connect_Button_clicked() {
    QString ip = ui->line_ip_Edit->text();
    QString port = ui->line_port_Edit->text();
    qDebug() << "ip: " << ip;
    qDebug() << "port: " << port;
    // 1、建立socket
    // PF_INET: IPv4, SOCK_STREAM: TCP
    int sockfd = socket(PF_INET, SOCK_STREAM, 0);
    assert(sockfd >= 0);
    // 2、socket绑定
    struct sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;   // ipv4
    inet_pton(AF_INET, ip.toStdString().c_str(), &address.sin_addr);
    address.sin_port = htons(8080);
    // 连接服务器
    if (::connect(sockfd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        QMessageBox::information(nullptr, "提示", "连接服务器失败");
    } else {
        QMessageBox::information(nullptr, "提示", "连接服务器成功！");
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::slot_rosShutdown() {
  qDebug() << "slot_rosShutdown";
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::slot_roboPos
/// \param pos item 坐标系下的robot pos
///
void MainWindow::slot_roboPos(QPointF pos) {
  QPointF ScenePos = roboItem_->mapToScene(pos);
  QPointF ViewPos = ui->mapViz->mapFromScene(ScenePos);
  QRect view_rect = ui->mapViz->viewport()->rect();   // 获取视图坐标范围
//  qDebug() << "ViewPos.x(): " << ViewPos.x() << ", y: " << ViewPos.y();
  if (ViewPos.x() < view_rect.width() * 0.2 || ViewPos.x() > view_rect.width() * 0.8
        || ViewPos.y() < view_rect.height() * 0.2 || ViewPos.y() > view_rect.height() * 0.8) {
    // 视图中心的item坐标系坐标
    QPointF scene_center = ui->mapViz->mapToScene(view_rect.center());   // 视图坐标系转换到scene坐标系
    QPointF scene_center_itempos = roboItem_->mapFromScene(scene_center);   // 再由scene坐标系转换到Item坐标系
    // 将view的中心移动到机器人上
    float dx = roboItem_->GetScale() * (scene_center_itempos.x() - pos.x());
    float dy = roboItem_->GetScale() * (scene_center_itempos.y() - pos.y());
    roboItem_->move(dx, dy);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_clicked() {
    if (frontend_process_->state() == QProcess::Running) {
        frontend_process_->terminate();
    }
    if (laser_process_->state() == QProcess::Running) {
        laser_process_->terminate();
    }
    if (hardware_process_->state() == QProcess::Running) {
        hardware_process_->terminate();
    }
    emit Quit();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::viewCenterFocusOnRobot
///
void MainWindow::viewCenterFocusOnRobot() {
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  const QPointF& robo_pos = roboItem_->GetRoboPos();
  float dx = roboItem_->GetScale() * (itemPoint.x() - robo_pos.x());
  float dy = roboItem_->GetScale() * (itemPoint.y() - robo_pos.y());
  roboItem_->move(dx, dy);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_return_clicked
///             将视图中心与机器人对齐
void MainWindow::on_pushButton_return_clicked() {
    viewCenterFocusOnRobot();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_plus_clicked
///             放大操作
void MainWindow::on_pushButton_plus_clicked() {
  // 获取放大操作的中心位置(相对于view窗口中心进行放大)
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  roboItem_->ChangeScale(1, roboItem_->GetScale() * itemPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_pushButton_minus_clicked
/// 缩小操作
void MainWindow::on_pushButton_minus_clicked() {
  // 获取缩小操作的中心位置(相对于view窗口中心进行缩小)
  QPoint viewCenter = ui->mapViz->viewport()->rect().center();   // 获取视图中心的坐标(视图坐标系)
  QPointF sceneCenter = ui->mapViz->mapToScene(viewCenter);   // 视图坐标系转换到scene坐标系
  QPointF itemPoint = roboItem_->mapFromScene(sceneCenter);   // 再由scene坐标系转换到Item坐标系
  roboItem_->ChangeScale(0, roboItem_->GetScale() * itemPoint);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_reset_clicked() {
  qt_ros_node_.SetReset();
}


void MainWindow::on_localgridmap_checkBox_stateChanged(int arg1) {
    if (arg1) {
//      qDebug() << "localgridmap on";
       qt_ros_node_.SetGridMapShowFlag(true);
       roboItem_->SetGridMapShow(true);
    } else {
//      qDebug() << "localgridmap off";
      qt_ros_node_.SetGridMapShowFlag(false);
      roboItem_->SetGridMapShow(false);
    }
}

