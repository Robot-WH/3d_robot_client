#include "PointCloudOpenGLWidget.h"
#include <QDebug>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

PointCloudOpenGLWidget::PointCloudOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    m_xRotate = -30.0;
    m_zRotate = 0;
    m_xTrans = 0.0;
    m_yTrans = 0.0;
    m_zTrans = 50;
    m_zoom = 45.0;
    roboPose_in_map_.setToIdentity();
    odom_to_map_.setToIdentity();
}

PointCloudOpenGLWidget::~PointCloudOpenGLWidget()
{
    makeCurrent();  // 这个函数调用通常用于将OpenGL上下文（context）设置为当前线程的上下文
    glDeleteBuffers(1, &m_VBO_MeshLine);
    glDeleteVertexArrays(1, &m_VAO_MeshLine);

    glDeleteBuffers(1, &m_VBO_Axis);
    glDeleteVertexArrays(1, &m_VAO_Axis);

    glDeleteBuffers(1, &m_VBO_Point);
    glDeleteVertexArrays(1, &m_VAO_Point);

    glDeleteBuffers(1, &m_VBO_GlobalLidarMap);
    glDeleteVertexArrays(1, &m_VAO_GlobalLidarMap);
    // 释放着色器
    m_shaderProgramMesh.release();
    m_shaderProgramAxis.release();
    // m_shaderProgramPoint.release();
    m_shaderProgramGlobalLidarMap.release();

    doneCurrent();
    qDebug() << __FUNCTION__;
}

/**
 * @brief PointCloudOpenGLWidget::SetGlobalLidarMap
 * @param map
 */
void PointCloudOpenGLWidget::SetGlobalLidarMap(sensor_msgs::PointCloud2ConstPtr map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*map, cloud);
  globalMapPointData.clear();
  globalMapPointData.reserve(cloud.size() * 4);
  qDebug() << "cloud.size()" << cloud.size();

  for(const auto& point : cloud)
  {
      globalMapPointData.push_back(point.x);
      globalMapPointData.push_back(point.y);
      globalMapPointData.push_back(point.z);
      globalMapPointData.push_back(1);
  }

  m_globalLidarMapCount = static_cast<GLsizei>(globalMapPointData.size() / 4);
  update();
}

/**
 * @brief PointCloudOpenGLWidget::SetRoboPose
 * @param pose
 */
void PointCloudOpenGLWidget::SetRoboPose(const QMatrix4x4& pose) {
  qDebug() << "SetRoboPose";
  odom_to_map_mt_.lock();
  roboPose_in_map_ = odom_to_map_ * pose;
  odom_to_map_mt_.unlock();
  update();
}

void PointCloudOpenGLWidget::SetOdomToMapTrans(const QMatrix4x4& pose) {
  odom_to_map_mt_.lock();
  odom_to_map_ = pose;
  odom_to_map_mt_.unlock();
}

/**
 * @brief PointCloudOpenGLWidget::updatePoints
 * @param points
 */
void PointCloudOpenGLWidget::updatePoints(const QVector<QVector3D> &points)
{
    m_pointData.clear();
    for(auto vector3D : points)
    {
        m_pointData.push_back(vector3D.x());
        m_pointData.push_back(vector3D.y());
        m_pointData.push_back(vector3D.z());
        m_pointData.push_back(1);
    }
}


/**
 * @brief PointCloudOpenGLWidget::initializeGL
 * initializeGL()函数在OpenGL渲染环境或窗口被初始化时调用一次，用于设置一些基本的渲染参数和加载着色器程序
 */
void PointCloudOpenGLWidget::initializeGL()
{
     // 这行代码用于初始化Qt提供的OpenGL函数。
    // 这通常是必要的，以确保Qt的OpenGL包装器能够正确调用底层的OpenGL函数
    initializeOpenGLFunctions();

    // enable depth_test
    // OpenGL支持深度测试（也称为Z测试），用于确定当多个图形对象重叠时哪个对象在前面（可见）。
    // 启用深度测试后，OpenGL会根据对象的深度值（即Z值）来确定哪个对象应该被渲染在前面。
    glEnable(GL_DEPTH_TEST);

    // 这部分代码加载并链接了用于渲染网格线的顶点着色器和片段着色器
    // link meshline shaders   vs文件为顶点着色器  fs为片段着色器
    m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Vertex,
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_mesh.vs");
    m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Fragment,
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_mesh.fs");
    m_shaderProgramMesh.link();

    // link coordinate axis shaders
    m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Vertex,
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_axis.vs");
    m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Fragment,
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_axis.fs");
    m_shaderProgramAxis.link();

    // link pointcloud shaders
    m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Vertex,     // 设置顶点着色器
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_point.vs");
    m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Fragment,      // 设置片段着色器
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_point.fs");
    m_shaderProgramPoint.link();

    m_shaderProgramGlobalLidarMap.addShaderFromSourceFile(QOpenGLShader::Vertex,     // 设置顶点着色器
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_point.vs");
    m_shaderProgramGlobalLidarMap.addShaderFromSourceFile(QOpenGLShader::Fragment,      // 设置片段着色器
            "/home/lwh/code/client_ws/src/robot_gui/resources/shader/shader_point.fs");
    m_shaderProgramGlobalLidarMap.link();

    // m_vertexCount = drawMeshline(2.0, 16);
    m_pointCount = drawPointdata(m_pointData);
    // qDebug() << "point_count" << m_pointCount;
    m_globalLidarMapCount = drawGlobalLidarMapData();
    drawCooraxis(1.0);
}

/**
 * @brief PointCloudOpenGLWidget::paintGL
 * 渲染OpenGL场景。每当需要重绘更新时自动调用
 */
void PointCloudOpenGLWidget::paintGL()
{
    // qDebug() << "paintGL";
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    /*
       为了将坐标从一个坐标系转换到另一个坐标系，需要用到几个转换矩阵，
       分别是模型(Model)、视图(View)、投影(Projection)三个矩阵。
    */
    QMatrix4x4 projection, view, model, model2;
    //透视矩阵变换
    projection.perspective(m_zoom, (float)width() / (float)height(), 1.0f, 10000.0f);

    // eye：摄像机位置  center：摄像机看的点位 up：摄像机上方的朝向
    view.lookAt(QVector3D(m_xTrans, m_yTrans, m_zTrans),
                              QVector3D(m_xTrans, m_yTrans, 1.0),
                              QVector3D(sin(m_zRotate), cos(m_zRotate), 0));

    // model.translate(m_xTrans, m_yTrans, 0.0);
    // model.rotate(m_xRotate, 1.0, 0.0, 0.0);
    // model.rotate(m_zRotate, 0.0, 0.0, 1.0);
    model.translate(0, 0, 0.0);
    model.rotate(0, 1.0, 0.0, 0.0);
    model.rotate(0, 0.0, 0.0, 1.0);

    model2.translate(10, 10, 0.0);
    model2.rotate(0, 1.0, 0.0, 0.0);
    model2.rotate(0, 0.0, 0.0, 1.0);
    // 对于每个着色器程序（网格、坐标轴和点云），都将其对应的变换矩阵设置为着色器程序的uniform变量。
    // 这样，着色器就可以使用这些矩阵来正确地转换和渲染顶点。
    m_shaderProgramMesh.bind();
    m_shaderProgramMesh.setUniformValue("projection", projection);
    m_shaderProgramMesh.setUniformValue("view", view);
    m_shaderProgramMesh.setUniformValue("model", model);

    m_shaderProgramAxis.bind();
    m_shaderProgramAxis.setUniformValue("projection", projection);
    m_shaderProgramAxis.setUniformValue("view", view);
    m_shaderProgramAxis.setUniformValue("model", roboPose_in_map_);

    m_shaderProgramPoint.bind();
    m_shaderProgramPoint.setUniformValue("projection", projection);
    m_shaderProgramPoint.setUniformValue("view", view);
    m_shaderProgramPoint.setUniformValue("model", model);

    m_shaderProgramGlobalLidarMap.bind();
    m_shaderProgramGlobalLidarMap.setUniformValue("projection", projection);
    m_shaderProgramGlobalLidarMap.setUniformValue("view", view);
    m_shaderProgramGlobalLidarMap.setUniformValue("model", model);

    //画网格
    // m_shaderProgramMesh.bind();
    // glBindVertexArray(m_VAO_MeshLine);    // 绑定网格的顶点数组对象（VAO）
    // glLineWidth(0.5f);    // 设置线宽为1.0
    // glDrawArrays(GL_LINES, 0, m_vertexCount);    // 使用glDrawArrays绘制网格线

    //画坐标轴
    m_shaderProgramAxis.bind();
    glBindVertexArray(m_VAO_Axis);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, 6);

    //画点云
    m_shaderProgramPoint.bind();
    glBindVertexArray(m_VAO_Point);
    glPointSize(1.0f);  // 设置点的大小
     // 为了让OpenGL知道我们的坐标和颜色值构成的到底是什么，OpenGL需要你去指定这些数据所表示的
     // 渲染类型。我们是希望把这些数据渲染成一系列的点？一系列的三角形？还是仅仅是一个长长的线？
     // 做出的这些提示叫做图元(Primitive)，任何一个绘制指令的调用都将把图元传递给OpenGL。
     // 这是其中的几个：GL_POINTS、GL_TRIANGLES、GL_LINE_STRIP。
    glDrawArrays(GL_POINTS, 0, m_pointCount);

    // 对m_VBO_GlobalLidarMap的数据进行更新
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_GlobalLidarMap);
    glBufferData(GL_ARRAY_BUFFER, globalMapPointData.size() * sizeof(float), &globalMapPointData[0], GL_STREAM_DRAW);
    //画全局地图
    m_shaderProgramGlobalLidarMap.bind();
    glBindVertexArray(m_VAO_GlobalLidarMap);
    glPointSize(1.0f);
    glDrawArrays(GL_POINTS, 0, m_globalLidarMapCount);
}

/**
 * @brief PointCloudOpenGLWidget::resizeGL
 * 设置OpenGL视区、投影等。每当小部件调整了大小时都会调用该视区
 * （并且当它第一次显示时也会调用，因为所有新创建的小部件都会自动获得一个调整大小的事件）
 * @param width
 * @param height
 */
void PointCloudOpenGLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
}

void PointCloudOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
    last_angle = std::atan2(lastPos.y() - 500, lastPos.x() - 500);
}

void PointCloudOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->pos().x() - lastPos.x();
    int dy = event->pos().y() - lastPos.y();
    // std::cout << "dx: " << dx << ", dy: " << dy << ",m_zRotate: " << m_zRotate << std::endl;
    // 处理鼠标左键按下时的移动
    if (event->buttons() & Qt::LeftButton)
    {
        float angle = std::atan2(event->pos().y() - 500, event->pos().x() - 500);
        float delta_angle = angle - last_angle;
        if (delta_angle < -M_PI) {
          delta_angle += 2 * M_PI;
        }
        if (delta_angle > M_PI) {
          delta_angle -= 2 * M_PI;
        }
        // std::cout << "delta_angle: " << delta_angle << "\n";
        m_zRotate -= delta_angle;
        // std::cout << "m_zRotate: " << m_zRotate << "\n";
         // 当调用update()时，它会将窗口或视图标记为需要被重绘，但并不会立即执行绘制操作。
        // 实际的绘制操作（通常包括调用paintGL()方法）是由Qt的事件循环在稍后的时间点安排的
        update();
        last_angle = angle;
    }
    else if (event->buttons() & Qt::MidButton)       // 处理鼠标中键按下时的移动
    {
        // 视角越低移动越慢，视角越高移动越块
        float delta_x = - dx * (m_zTrans / 1000);
        float delta_y = dy * (m_zTrans / 1000);
        // 如果鼠标中键被按下（event->buttons() & Qt::MidButton），则根据鼠标在x轴和y轴上的移动距离来平移模型。
        // 注意这里平移的系数（0.1）是另一个常数，可以根据需要进行调整。
        float delta_x_trans = delta_x * cos(m_zRotate) + delta_y * sin(m_zRotate);
        float delta_y_trans = delta_y * cos(m_zRotate) - delta_x * sin(m_zRotate);
        m_xTrans = m_xTrans + delta_x_trans;
        m_yTrans = m_yTrans + delta_y_trans;
        update();
    }
    lastPos = event->pos();
}

void PointCloudOpenGLWidget::wheelEvent(QWheelEvent *event)
{
    auto scroll_offest = event->angleDelta().y() / 120;
    // m_zoom = m_zoom - (float)scroll_offest;
    m_zTrans -= (float)scroll_offest * (m_zTrans / 10);

    // if (m_zoom < 1.0f)    /* 放大限制 */
    // {
    //     m_zoom = 1.0f;
    // }

    // if (m_zoom > 80.0f)
    // {
    //     m_zoom = 80.0f;
    // }

    if (m_zTrans < 1.0f)    /* 放大限制 */
    {
        m_zTrans = 1.0f;
    }

    if (m_zTrans > 10000.0f)
    {
        m_zTrans = 10000.0f;
    }
    update();
}

unsigned int PointCloudOpenGLWidget::drawMeshline(float size, int count)
{
    std::vector<float> mesh_vertexs;
    unsigned int vertex_count = 0;

    float start = count * (size / 2);
    float posX = start, posZ = start;

    for (int i = 0; i <= count; ++i)
    {
        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);

        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);

        posX = posX - size;
        posZ = posZ - size;
    }

    glGenVertexArrays(1, &m_VAO_MeshLine);   // 生成VAO
    glGenBuffers(1, &m_VBO_MeshLine);    // 生成VBO
    // 绑定VAO
    glBindVertexArray(m_VAO_MeshLine);
    // 绑定VBO
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_MeshLine);
    // 将数据传送到VBO缓存中
    glBufferData(GL_ARRAY_BUFFER, mesh_vertexs.size() * sizeof(float), &mesh_vertexs[0], GL_STATIC_DRAW);
    // 设置VAO
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    // 解绑
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    vertex_count = (int)mesh_vertexs.size() / 3;
    return vertex_count;
}

void PointCloudOpenGLWidget::drawCooraxis(float length)
{
    std::vector<float> axis_vertexs =
    {
        //x,y ,z ,r, g, b
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        length, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, length, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, length, 0.0, 0.0, 1.0,
    };

    glGenVertexArrays(1, &m_VAO_Axis);
    glGenBuffers(1, &m_VBO_Axis);

    glBindVertexArray(m_VAO_Axis);

    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Axis);
    glBufferData(GL_ARRAY_BUFFER, axis_vertexs.size() * sizeof(float), &axis_vertexs[0], GL_STATIC_DRAW);
    // 位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    // 颜色属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    // 解绑   VBO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // 解绑   VAO
    glBindVertexArray(0);
}

/**
 * @brief PointCloudOpenGLWidget::drawPointdata
 *  这个函数接收一个std::vector<float>类型的引用pointVertexs，这个向量预计包含了点云数据的顶点信息（每个点可能是由位置坐标和颜色信息组成的）。
 *   函数返回一个unsigned int类型的值，表示点的数量
 * @param pointVertexs
 * @return
 */
unsigned int PointCloudOpenGLWidget::drawPointdata(std::vector<float> &pointVertexs)
{
    /************************************** 生成顶点数组对象 (VAO) *********************************/
    // 使用 VAO 的主要好处是性能，可以将每个物体的顶点属性设置存储在一个 VAO 中，
    // 并在渲染时通过绑定不同的 VAO 来快速切换这些设置，从而避免了重复的状态设置开销。
    // 第一个参数1表示要生成一个顶点数组对象。
    // 第二个参数&m_VAO_Point是指针，用来存储新生成的顶点数组对象的名称（ID）
    glGenVertexArrays(1, &m_VAO_Point);
    glBindVertexArray(m_VAO_Point);   // 绑定这个VAO，后续的顶点属性设置（如使用 glVertexAttribPointer）会修改该 VAO 的状态

    /*************************************生成顶点缓冲对象 (VBO)***********************************/
    // 创建一个节点缓冲对象
    glGenBuffers(1, &m_VBO_Point);
    // 顶点缓冲对象的缓冲类型是GL_ARRAY_BUFFER，把新创建的缓冲绑定到GL_ARRAY_BUFFER目标
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Point);   // m_VBO_Point 会和m_VAO_Point 绑定在一起

    /*************************************上传顶点数据到VBO***************************************/
    // 将顶点数据从pointVertexs上传到之前绑定的VBO中。数据被标记为GL_STATIC_DRAW`，表示数据不会经常改变。
    // GL_STATIC_DRAW：数据不会或几乎不会改变。
    // GL_DYNAMIC_DRAW：数据会被改变很多。
    // GL_STREAM_DRAW ：数据每次绘制时都会改变。
    // 如果，比如说一个缓冲中的数据将频繁被改变，那么使用的类型就是GL_DYNAMIC_DRAW或GL_STREAM_DRAW，
    // 这样就能确保显卡把数据放在能够高速写入的内存部分。
    glBufferData(GL_ARRAY_BUFFER, pointVertexs.size() * sizeof(float), &pointVertexs[0], GL_STATIC_DRAW);

    /****************设置顶点属性，设置VAO   告诉opengl如何解析缓存中的数据*************************/
    // 位置属性
    // 第一个参数指定我们要配置的顶点属性，在顶点着色器中使用layout(location = 0)，
    // 希望把数据传递到这一个顶点的位置属性中，所以设置为0
    // 第二个参数指定顶点属性的大小。顶点属性是一个vec3，它由3个值组成，所以大小是3
    // 第三个参数指定数据的类型，这里是GL_FLOAT(GLSL中vec*都是由浮点数值组成的)
    // 下个参数定义我们是否希望数据被标准化(Normalize)。如果我们设置为GL_TRUE，所有数据都会被映射到0（对于有符号型signed数据是-1）到1之间。
    // 第五个参数叫做步长(Stride)，它告诉我们在连续的顶点属性组之间的内存间隔。由于每个顶点属性组存了4个float数据，所以把步长设置为4 * sizeof(float)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);   // 告诉OpenGL该如何解析顶点数据
    glEnableVertexAttribArray(0);
    // 颜色属性
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    /****************************************解绑VBO和VAO*******************************/
    //  将上下文的GL_ARRAY_BUFFER对象设回默认的ID-0,相当于与所创建的VAO和VBO解绑
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return (unsigned int)pointVertexs.size() / 4;
}

/**
 * @brief PointCloudOpenGLWidget::drawGlobalLidarMapData
 * @return
 */
unsigned int PointCloudOpenGLWidget::drawGlobalLidarMapData()
{
    glGenVertexArrays(1, &m_VAO_GlobalLidarMap);
    glBindVertexArray(m_VAO_GlobalLidarMap);

    glGenBuffers(1, &m_VBO_GlobalLidarMap);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_GlobalLidarMap);
    glBufferData(GL_ARRAY_BUFFER, globalMapPointData.size() * sizeof(float), &globalMapPointData[0], GL_STREAM_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);   // 告诉OpenGL该如何解析顶点数据
    glEnableVertexAttribArray(0);
    // 颜色属性
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return (unsigned int)globalMapPointData.size() / 4;
}
