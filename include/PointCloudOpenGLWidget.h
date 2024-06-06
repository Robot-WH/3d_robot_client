#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QMouseEvent>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>

/**
 * @brief The PointCloudOpenGLWidget class
 * QOpenGLWidget代替GLFW， QOpenGLFunctions_X_X_Core代替GLAD
 * QOpenGLFunctions_X_X_Core提供OpenGL X.X版本核心模式的所有功能。是对OpenGL函数的封装
 */
class PointCloudOpenGLWidget : public QOpenGLWidget, public QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    PointCloudOpenGLWidget(QWidget *parent = 0);
    ~PointCloudOpenGLWidget();
    void updatePoints(const QVector<QVector3D> &points);
    void SetGlobalLidarMap(sensor_msgs::PointCloud2ConstPtr map);
    void SetLidarStablePoint(sensor_msgs::PointCloud2ConstPtr points);
    void SetRoboPose(const QMatrix4x4& pose);
    void SetOdomToMapTrans(const QMatrix4x4& pose);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event);

    virtual unsigned int drawMeshline(float size, int count);
    virtual void drawCooraxis(float length);
    virtual unsigned int drawLidarStablePointdata();
    virtual unsigned int drawGlobalLidarMapData();

protected:
    QOpenGLShaderProgram m_shaderProgramMesh;
    QOpenGLShaderProgram m_shaderProgramAxis;
    QOpenGLShaderProgram m_shaderProgramLidarStablePoint;
    QOpenGLShaderProgram m_shaderProgramGlobalLidarMap;

    unsigned int m_VBO_MeshLine;
    unsigned int m_VAO_MeshLine;

    unsigned int m_VBO_Axis;
    unsigned int m_VAO_Axis;

    unsigned int m_VBO_Point;
    unsigned int m_VAO_Point;

    unsigned int m_VBO_GlobalLidarMap;
    unsigned int m_VAO_GlobalLidarMap;

    std::vector<float> m_lidarStablePointData;
    std::vector<float> globalMapPointData;
    unsigned int m_globalLidarMapCount;
    unsigned int m_lidarStablePointCount;

    unsigned int m_vertexCount;

    float m_xRotate;
    float m_zRotate;
    float m_xTrans;
    float m_yTrans;
    float m_zTrans;
    float m_zoom;
    float last_angle;
    QPoint   lastPos;
    QMatrix4x4 roboPose_in_map_;
    QMatrix4x4 odom_to_map_;
    std::mutex odom_to_map_mt_;
};
