#include <QMovie>
#include <QMessageBox>
#include <QDebug>
#include <QNetworkInterface>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cstdlib>

#include "MainWindow.h"
#include "ui_MainWindow.h"    // build/my_gui下 

////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(int argc, char **argv, QWidget *parent) :
    QWidget(parent), ui(new Ui::MainWindow), qt_ros_node_(argc, argv) {
  initUI();    // 初始化UI
  initParm(); // 初始化参数
  //创建item
  roboItem_ = new ros_qt::roboItem();
  // 系统信息输出
  ui->msg_output->setReadOnly(true);
  // 设置最大显示文本容量为100个文本块
  ui->msg_output->document()->setMaximumBlockCount(100);

  frontend_process_ = new QProcess;
  backend_process_ = new QProcess;
  rviz_process_ = new QProcess;
  // 将ROS节点的输出重定向到QTextBrowser控件中
  QObject::connect(frontend_process_, &QProcess::readyReadStandardOutput, [&]() {
      QString output = frontend_process_->readAllStandardOutput();
      ui->msg_output->append(output);
  });
  // 设置QProcess的标准输出通道为MergedChannels
  frontend_process_->setProcessChannelMode(QProcess::MergedChannels);
  // 信号，槽连接
  connection();

  QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
  palette.setColor(QPalette::WindowText, Qt::red);  // 设置文字颜色为红色
  ui->label_status->setPalette(palette);  // 应用新的调色板
  ui->label_status->setText("ROS主机未连接");
  ui->label_status_img->setPixmap(QPixmap("://images/false.png"));  // 设置背景图片
  ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
  ui->label_status_img->show();
  // ui->bringup_button->setEnabled(false);
  qt_ros_node_.SetOpenGLWidget(ui->openGLWidget);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {
  qDebug() << "~MainWindow()";
  if (frontend_process_->state() == QProcess::Running) {
      frontend_process_->terminate();
  }
  if (backend_process_->state() == QProcess::Running) {
      backend_process_->terminate();
  }
  if (rviz_process_->state() == QProcess::Running) {
      rviz_process_->terminate();
  }
  delete ui;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::initUI() {
  ui->setupUi(this);
  //  Qt::Window: 指示窗口是一个普通窗口，拥有标题栏和边框。
  //  Qt::WindowTitleHint: 显示窗口标题栏。
  //  Qt::CustomizeWindowHint: 禁用默认的窗口装饰，即关闭、最大化和最小化按钮
   this->setWindowFlags(Qt::Window);
 // this->setWindowFlags(Qt::FramelessWindowHint);  //去掉标题栏

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
//  QRectF visibleRect = ui->mapViz->mapToScene(ui->mapViz->viewport()->geometry()).boundingRect();
//  QPointF visibleRect = ui->mapViz->mapToScene(0, 0);
//  QPointF sceneCenter = ui->mapViz->mapToScene(ui->mapViz->viewport()->rect().topLeft());
//  qDebug() << "sceneCenter w:" << sceneCenter.x() << ", h: " << sceneCenter.y();
  ui->tabWidget->setFixedWidth(400);
//  ui->groupBox_10->setFixedWidth(300);
//  ui->groupBox_6->setFixedHeight(250);
  // 放大缩小设置
//  ui->groupBox_5->setFixedWidth(40);
////  ui->groupBox_5->setStyleSheet("QGroupBox { padding: 2px; }");
//  ui->groupBox_5->setStyleSheet("QGroupBox { border: none; }");

  // ui->server_connect_Button->setFixedHeight(60);
  // ui->server_connect_Button->setFixedWidth(60);
//  // 退出
//  ui->pushButton->setFixedHeight(50);
//  ui->pushButton->setFixedWidth(100);
  // ui->image_label_0->setFixedSize(300, 200);
  ui->listWidget->setFixedHeight(100);
  ui->listWidget_2->setFixedHeight(100);
  ui->listWidget_2->setFixedWidth(80);

  ui->groupBox_2->setFixedHeight(90);
  ui->groupBox_3->setFixedHeight(90);
  // ui->verticalLayout_2->setFixedHeight(ui->groupBox_3->height());
  // ui->verticalLayout_2->setFixedWidth(ui->groupBox_3->width());
  // ui->verticalLayout_2->SetFixedSize();
  // this->setFixedHeight(ui->groupBox_3->height());
  // this->setFixedWidth(ui->groupBox_3->width());
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
  connect(&qt_ros_node_, &ros_qt::QNode::showImage, roboItem_,
          [&](const int& num, const QImage& img) {
  });

  // 机器人pose -> map
  connect(&qt_ros_node_, &ros_qt::QNode::updateRoboPose, roboItem_,
          &ros_qt::roboItem::paintRoboPos);
  // radioButton
  // QRadioButton 有一个状态切换的信号 toggled，即该信号在状态切换时发送；
  // 设置激光雷达颠倒
  connect(ui->radioButton_3, &QRadioButton::toggled, [=](bool isChecked){
      if (isChecked == true) {
        roboItem_->SetLaserInverted(true);
      } else {
        roboItem_->SetLaserInverted(false);
      }
  });
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool MainWindow::ConnectRosMaster() {
  // 读取本机的IP
  QString hostUrl = getLocalIpAddress();
  if (hostUrl.isEmpty()) {
    // hostUrl = "NanoPi-M4";
    // qDebug() << "hostUrl.isEmpty() " << getenv("HOSTNAME");
    hostUrl = getenv("HOSTNAME");
  }
  QString masterUrl = "http://" + hostUrl + ":11311";
  // qDebug() << "masterUrl: " << masterUrl;
  // 连接ros master
  if (!qt_ros_node_.init(masterUrl.toStdString(), hostUrl.toStdString())) {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::red);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS 连接失败");
    ui->label_status_img->setPixmap(QPixmap("://images/false.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
    ui->bringup_button->setEnabled(false);
    return false;
  } else {
    QPalette palette = ui->label_status->palette();  // 获取QLabel的调色板
    palette.setColor(QPalette::WindowText, Qt::darkGreen);  // 设置文字颜色为红色
    ui->label_status->setPalette(palette);  // 应用新的调色板
    ui->label_status->setText("ROS Master 连接成功");
    ui->label_status_img->setPixmap(QPixmap("://images/ok.png"));  // 设置背景图片
    ui->label_status_img->setScaledContents(true);  // 设置QLabel的内容自适应缩放
    ui->label_status_img->show();
    ui->bringup_button->setEnabled(true);
//    //初始化视频订阅的显示
//    initVideos();
//    //显示话题列表
//    initTopicList();
//    initOthers();
  }
//  ReadSettings();
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_bringup_button_clicked() {
  // startDetached 启动的进程不能进行通信，也不能输出重定向，但是，主线程退出后不会影响该子进程的执行
  // frontend_process_->startDetached("roslaunch", QStringList() << "calib_fusion_2d" << "frontend.launch");
  //   启动roslaunch命令
  frontend_process_->start("roslaunch", QStringList() << "lwio" << "kitti_advanced_lio_velodyne_icp_timedIvox.launch");
  backend_process_->start("roslaunch", QStringList() << "lifelong_backend" << "backend.launch");
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
    // QString ip = ui->line_ip_Edit->text();
    // QString port = ui->line_port_Edit->text();
    // qDebug() << "ip: " << ip;
    // qDebug() << "port: " << port;
    // // 1、建立socket
    // // PF_INET: IPv4, SOCK_STREAM: TCP
    // int sockfd = socket(PF_INET, SOCK_STREAM, 0);
    // assert(sockfd >= 0);
    // // 2、socket绑定
    // struct sockaddr_in address;
    // std::memset(&address, 0, sizeof(address));
    // address.sin_family = AF_INET;   // ipv4
    // inet_pton(AF_INET, ip.toStdString().c_str(), &address.sin_addr);
    // address.sin_port = htons(8080);
    // // 连接服务器
    // if (::connect(sockfd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    //     QMessageBox::information(nullptr, "提示", "连接服务器失败");
    // } else {
    //     QMessageBox::information(nullptr, "提示", "连接服务器成功！");
    // }
    ConnectRosMaster();
    // 延时
    QTime t;
    t.start();
    while(t.elapsed()<500)
          QCoreApplication::processEvents();
    // 连接后，读取工作空间
    qt_ros_node_.pubGetWorkSpaceCmd();
    // 最多等待500ms
    t.start();
    while(t.elapsed() < 500) {
      if (qt_ros_node_.isWorkspaceUpdate()) {
        const auto& workspace = qt_ros_node_.getWorkspace();

        for (const auto& name : workspace) {
          ui->listWidget->addItem(QString::fromStdString(name));
        }
        break;
      }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::slot_rosShutdown() {
  qDebug() << "slot_rosShutdown";
}



////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_clicked() {
    if (frontend_process_->state() == QProcess::Running) {
        frontend_process_->terminate();
    }
    if (backend_process_->state() == QProcess::Running) {
        backend_process_->terminate();
    }
    emit Quit();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_reset_clicked() {
  // qDebug() << "pushButton_reset";
  qt_ros_node_.SetReset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_localgridmap_checkBox_stateChanged(int arg1) {
    if (arg1) {
    //  qDebug() << "localgridmap on";
       qt_ros_node_.SetGridMapShowFlag(true);
       roboItem_->SetGridMapShow(true);
    } else {
    //  qDebug() << "localgridmap off";
      qt_ros_node_.SetGridMapShowFlag(false);
      roboItem_->SetGridMapShow(false);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief 地图空间双击选择
/// \param item
///
void MainWindow::on_listWidget_itemDoubleClicked(QListWidgetItem *item) {
    std::vector<uint16_t> traj_id;
    // 发送设置space的服务
    qt_ros_node_.setSpaceCall(item->text(), traj_id);
    ui->label_6->setText(QString::number(traj_id.size()));
    ui->listWidget_2->clear();

    for (const auto& id : traj_id) {
      ui->listWidget_2->addItem(QString::number(id));
    }

    ui->label_9->setText(item->text());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief 保存轨迹
///
void MainWindow::on_pushButton_4_clicked() {
  std::vector<uint16_t> traj_id;
  qt_ros_node_.saveTrajCall(QString(""), traj_id);
  QMessageBox::information(nullptr, "提示", "保存成功！");

  ui->label_6->setText(QString::number(traj_id.size()));
  ui->listWidget_2->clear();

  for (const auto& id : traj_id) {
    ui->listWidget_2->addItem(QString::number(id));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief 轨迹列表双击
/// \param item
///
void MainWindow::on_listWidget_2_itemDoubleClicked(QListWidgetItem *item) {
  uint8_t success = 0;
  uint16_t num = item->text().toUInt();
  // qDebug() << "num: " << num;
  qt_ros_node_.setTrajCall(num, success);
  if (!success) {
    QMessageBox::information(nullptr, "错误", "轨迹不存在！");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_radioButton_8_clicked
/// 工作模式-长期建图与定位模式选择
void MainWindow::on_radioButton_8_clicked() {
  uint8_t res = 0;
  qt_ros_node_.setWorkModeCall(1, res);
  if (res) {
    QMessageBox::information(nullptr, "提示", "设置为长期建图与定位模式");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_radioButton_10_clicked
/// 工作模式-定位模式选择
void MainWindow::on_radioButton_10_clicked() {
  uint8_t res = 0;
  qt_ros_node_.setWorkModeCall(2, res);
  if (res) {
    QMessageBox::information(nullptr, "提示", "设置为纯定位模式");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief MainWindow::on_radioButton_9_clicked
///工作模式-建图模式选择
void MainWindow::on_radioButton_9_clicked() {
  uint8_t res = 0;
  qt_ros_node_.setWorkModeCall(3, res);
  if (res) {
    QMessageBox::information(nullptr, "提示", "设置为纯建图模式");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::on_pushButton_7_clicked()
{
  rviz_process_->start("roslaunch", QStringList() << "lifelong_backend" << "rviz.launch");
}

