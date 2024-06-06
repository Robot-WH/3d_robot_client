#ifndef MainWindow_H
#define MainWindow_H

#include <QWidget>
#include <QProcess>
#include <QListWidgetItem>
#include "qnode.hpp"
#include "roboItem.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
  explicit MainWindow(int argc, char **argv, QWidget *parent = nullptr);
  ~MainWindow();
signals:
   void Quit();

protected:
  /**
   * @brief getLocalIpAddress 获取本机的IP地址
   * @return
   */
  QString getLocalIpAddress();

  /**
   * @brief MainWindow::connection 信号与槽的连接
   */
  void connection();

  /**
   * @brief MainWindow::initUI 初始化UI
   */
  void initUI();

  /**
   * @brief MainWindow::initParm 初始化各种参数
   */
  void initParm();

  /**
   * @brief MainWindow::bringUpQtRosNode
   */
  bool ConnectRosMaster();

private slots:
  void on_radioButton_9_clicked();

private slots:
  void on_radioButton_10_clicked();

private slots:
  void on_radioButton_8_clicked();

private slots:
  void on_listWidget_2_itemDoubleClicked(QListWidgetItem *item);

private slots:
  void on_pushButton_4_clicked();

private slots:
  void on_listWidget_itemDoubleClicked(QListWidgetItem *item);

private slots:
  void on_localgridmap_checkBox_stateChanged(int arg1);

private slots:
  void on_pushButton_reset_clicked();

private slots:
  void on_pushButton_clicked();
  /**
   * @brief MainWindow::on_bringup_button_clicked 启动机器人系统
   */
  void on_bringup_button_clicked();

  /**
   * @brief MainWindow::on_server_connect_Button_clicked
   *                连接服务器
   */
  void on_server_connect_Button_clicked();

  void slot_rosShutdown();

  void on_pushButton_7_clicked();

private:
  Ui::MainWindow *ui;
  QProcess *backend_process_;
  QProcess *frontend_process_;
  QProcess *rviz_process_;
  ros_qt::QNode qt_ros_node_;

  ros_qt::roboItem *roboItem_ = NULL;
};

#endif // MainWindow_H
