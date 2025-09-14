#include "slam_gui/visualization_widget.h"
#include <QMessageBox>
#include <QDebug>

VisualizationWidget::VisualizationWidget(QWidget* parent)
    : QWidget(parent)
    , main_layout_(nullptr)
    , top_layout_(nullptr)
    , splitter_(nullptr)
    , vtk_widget_(nullptr)
    , vtk_native_widget_(nullptr)
    , vtk_available_(false)
    , control_panel_(nullptr)
    , data_updated_(false)
{
    setupUI();
    setupVTK();
    setupControlPanel();
    connectSignals();
    
    // 初始化定时器
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &VisualizationWidget::updateVisualization);
    update_timer_->start(33); // ~30 FPS
    
    // 初始化点云
    current_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

VisualizationWidget::~VisualizationWidget()
{
    if (update_timer_) {
        update_timer_->stop();
    }
}

void VisualizationWidget::setupUI()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(2, 2, 2, 2);
    
    // 创建分割器
    splitter_ = new QSplitter(Qt::Horizontal, this);
    main_layout_->addWidget(splitter_);
    
    setLayout(main_layout_);
}

void VisualizationWidget::setupVTK()
{
    try {
        // 创建VTK渲染窗口
        render_window_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        renderer_ = vtkSmartPointer<vtkRenderer>::New();
        render_window_->AddRenderer(renderer_);
        
        // 创建VTK widget
        vtk_native_widget_ = new QVTKOpenGLNativeWidget(this);
        vtk_native_widget_->setRenderWindow(render_window_);
        vtk_native_widget_->setMinimumSize(400, 300);
        vtk_widget_ = vtk_native_widget_;
        vtk_available_ = true;
        
        // 设置渲染器
        renderer_->SetBackground(0.1, 0.1, 0.15); // 深色背景
        
        // 初始化VTK actors
        point_cloud_actor_ = vtkSmartPointer<vtkActor>::New();
        trajectory_actor_ = vtkSmartPointer<vtkActor>::New();
        keyframes_actor_ = vtkSmartPointer<vtkActor>::New();
        loop_closures_actor_ = vtkSmartPointer<vtkActor>::New();
        
        // 添加actors到渲染器
        renderer_->AddActor(point_cloud_actor_);
        renderer_->AddActor(trajectory_actor_);
        renderer_->AddActor(keyframes_actor_);
        renderer_->AddActor(loop_closures_actor_);
        
        // 设置坐标轴和光照
        setupCoordinateAxes();
        setupLighting();
        
        // 初始化相机
        auto camera = renderer_->GetActiveCamera();
        camera->SetPosition(0, 0, 50);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
        
        splitter_->addWidget(vtk_widget_);
        
        qDebug() << "VTK可视化系统初始化成功";
        
    } catch (const std::exception& e) {
        qWarning() << "VTK初始化失败，使用占位符:" << e.what();
        
        // 降级到简化版本
        vtk_widget_ = new QWidget(this);
        vtk_native_widget_ = nullptr;
        vtk_available_ = false;
        vtk_widget_->setMinimumSize(400, 300);
        vtk_widget_->setStyleSheet("background-color: #2a2a2a; border: 1px solid #404040;");
        
        QVBoxLayout* layout = new QVBoxLayout(vtk_widget_);
        QLabel* error_label = new QLabel("VTK初始化失败，请检查依赖库", vtk_widget_);
        error_label->setAlignment(Qt::AlignCenter);
        error_label->setStyleSheet("color: #ff6666; font-size: 14px;");
        layout->addWidget(error_label);
        
        splitter_->addWidget(vtk_widget_);
    }
}

void VisualizationWidget::setupControlPanel()
{
    control_panel_ = new QGroupBox("显示控制", this);
    control_panel_->setMaximumWidth(200);
    control_panel_->setMinimumWidth(180);
    
    QVBoxLayout* control_layout = new QVBoxLayout(control_panel_);
    
    // 显示选项
    show_point_cloud_cb_ = new QCheckBox("显示点云");
    show_point_cloud_cb_->setChecked(settings_.show_point_cloud);
    control_layout->addWidget(show_point_cloud_cb_);
    
    show_trajectory_cb_ = new QCheckBox("显示轨迹");
    show_trajectory_cb_->setChecked(settings_.show_trajectory);
    control_layout->addWidget(show_trajectory_cb_);
    
    show_keyframes_cb_ = new QCheckBox("显示关键帧");
    show_keyframes_cb_->setChecked(settings_.show_keyframes);
    control_layout->addWidget(show_keyframes_cb_);
    
    show_loop_closures_cb_ = new QCheckBox("显示回环");
    show_loop_closures_cb_->setChecked(settings_.show_loop_closures);
    control_layout->addWidget(show_loop_closures_cb_);
    
    control_layout->addWidget(new QLabel("颜色模式:"));
    color_mode_combo_ = new QComboBox();
    color_mode_combo_->addItems({"高度着色", "强度着色", "时间着色", "法向量着色", "自定义颜色"});
    color_mode_combo_->setCurrentIndex(static_cast<int>(settings_.point_cloud_color_mode));
    control_layout->addWidget(color_mode_combo_);
    
    control_layout->addWidget(new QLabel("点大小:"));
    point_size_spin_ = new QDoubleSpinBox();
    point_size_spin_->setRange(0.1, 10.0);
    point_size_spin_->setSingleStep(0.1);
    point_size_spin_->setValue(settings_.point_size);
    control_layout->addWidget(point_size_spin_);
    
    control_layout->addWidget(new QLabel("最大点数:"));
    max_points_spin_ = new QSpinBox();
    max_points_spin_->setRange(1000, 2000000);
    max_points_spin_->setSingleStep(10000);
    max_points_spin_->setValue(settings_.max_points_display);
    control_layout->addWidget(max_points_spin_);
    
    // 控制按钮
    control_layout->addWidget(new QLabel(""));
    
    reset_view_btn_ = new QPushButton("重置视角");
    control_layout->addWidget(reset_view_btn_);
    
    center_view_btn_ = new QPushButton("居中显示");
    control_layout->addWidget(center_view_btn_);
    
    screenshot_btn_ = new QPushButton("截图保存");
    control_layout->addWidget(screenshot_btn_);
    
    control_layout->addStretch();
    
    splitter_->addWidget(control_panel_);
    splitter_->setSizes({800, 200});
}

void VisualizationWidget::connectSignals()
{
    if (show_point_cloud_cb_) {
        connect(show_point_cloud_cb_, &QCheckBox::toggled, this, &VisualizationWidget::onPointCloudToggled);
    }
    if (show_trajectory_cb_) {
        connect(show_trajectory_cb_, &QCheckBox::toggled, this, &VisualizationWidget::onTrajectoryToggled);
    }
    if (show_keyframes_cb_) {
        connect(show_keyframes_cb_, &QCheckBox::toggled, this, &VisualizationWidget::onKeyframesToggled);
    }
    if (show_loop_closures_cb_) {
        connect(show_loop_closures_cb_, &QCheckBox::toggled, this, &VisualizationWidget::onLoopClosuresToggled);
    }
    if (color_mode_combo_) {
        connect(color_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
                this, &VisualizationWidget::onColorModeChanged);
    }
    if (point_size_spin_) {
        connect(point_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
                this, &VisualizationWidget::onPointSizeChanged);
    }
    if (max_points_spin_) {
        connect(max_points_spin_, QOverload<int>::of(&QSpinBox::valueChanged), 
                this, &VisualizationWidget::onMaxPointsChanged);
    }
    if (reset_view_btn_) {
        connect(reset_view_btn_, &QPushButton::clicked, this, &VisualizationWidget::onResetView);
    }
}

void VisualizationWidget::updateSLAMData(const SLAMData& data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_slam_data_ = data;
    data_updated_ = true;
}

void VisualizationWidget::setVisualizationSettings(const VisualizationSettings& settings)
{
    settings_ = settings;
    // 更新UI控件状态
    if (show_point_cloud_cb_) show_point_cloud_cb_->setChecked(settings_.show_point_cloud);
    if (show_trajectory_cb_) show_trajectory_cb_->setChecked(settings_.show_trajectory);
    // ... 更新其他控件
}

VisualizationSettings VisualizationWidget::getVisualizationSettings() const
{
    return settings_;
}

void VisualizationWidget::updateVisualization()
{
    if (!data_updated_) return;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    try {
        // 更新点云 - 优先显示全局持久化地图，然后是SLAM当前帧，最后是原始雷达数据
        if (settings_.show_point_cloud) {
            if (current_slam_data_.world_cloud) {
                // 显示持久化全局地图 (推荐显示)
                updatePointCloudVisualization(current_slam_data_.world_cloud);
            } else if (current_slam_data_.point_cloud) {
                // 显示SLAM处理后的当前帧点云
                updatePointCloudVisualization(current_slam_data_.point_cloud);
            } else if (current_slam_data_.raw_point_cloud) {
                // 显示原始雷达点云
                updatePointCloudVisualization(current_slam_data_.raw_point_cloud);
            }
        }
        
        // 更新轨迹
        if (current_slam_data_.path && settings_.show_trajectory) {
            updateTrajectoryVisualization(current_slam_data_.path);
        }
        
        // 更新关键帧
        if (settings_.show_keyframes) {
            updateKeyframesVisualization();
        }
        
        // 更新回环
        if (current_slam_data_.loop_closures && settings_.show_loop_closures) {
            updateLoopClosuresVisualization(current_slam_data_.loop_closures);
        }
        
        // 简化版本：暂时跳过渲染
        // TODO: 实现VTK渲染
        
    } catch (const std::exception& e) {
        // 忽略可视化更新错误，避免影响主程序
        static int error_count = 0;
        if (++error_count < 5) { // 只显示前5个错误
            qWarning() << "可视化更新错误:" << e.what();
        }
    }
    
    data_updated_ = false;
}

void VisualizationWidget::updatePointCloudVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg)
{
    if (!cloud_msg || !vtk_available_ || !render_window_) return;
    
    try {
        // 转换ROS消息到PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
        
        // 降采样
        if (pcl_cloud->size() > static_cast<size_t>(settings_.max_points_display)) {
            pcl_cloud = downsamplePointCloud(pcl_cloud);
        }
        
        current_point_cloud_ = pcl_cloud;
        
        // 创建VTK点云数据
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");
        
        for (const auto& point : pcl_cloud->points) {
            points->InsertNextPoint(point.x, point.y, point.z);
            
            // 根据颜色模式设置颜色
            auto color = getPointColor(point);
            colors->InsertNextTuple3(color[0], color[1], color[2]);
        }
        
        // 创建点云polydata
        auto polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->SetPoints(points);
        polydata->GetPointData()->SetScalars(colors);
        
        // 创建顶点
        auto vertices = vtkSmartPointer<vtkCellArray>::New();
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            vertices->InsertNextCell(1, &i);
        }
        polydata->SetVerts(vertices);
        
        // 创建mapper和actor
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        mapper->SetColorModeToDirectScalars();
        
        point_cloud_actor_->SetMapper(mapper);
        point_cloud_actor_->GetProperty()->SetPointSize(settings_.point_size);
        
        // 刷新渲染
        render_window_->Render();
        
        qDebug() << "点云可视化更新:" << pcl_cloud->size() << "个点";
        
    } catch (const std::exception& e) {
        qWarning() << "点云可视化更新失败:" << e.what();
    }
}

void VisualizationWidget::updateTrajectoryVisualization(const nav_msgs::msg::Path::SharedPtr& path_msg)
{
    if (!path_msg || !vtk_available_ || !render_window_ || path_msg->poses.empty()) return;
    
    try {
        // 创建轨迹线条数据
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto lines = vtkSmartPointer<vtkCellArray>::New();
        
        // 添加轨迹点
        for (size_t i = 0; i < path_msg->poses.size(); ++i) {
            const auto& pose = path_msg->poses[i].pose.position;
            points->InsertNextPoint(pose.x, pose.y, pose.z);
            
            if (i > 0) {
                lines->InsertNextCell(2);
                lines->InsertCellPoint(i - 1);
                lines->InsertCellPoint(i);
            }
        }
        
        // 创建polydata
        auto polydata = vtkSmartPointer<vtkPolyData>::New();
        polydata->SetPoints(points);
        polydata->SetLines(lines);
        
        // 创建mapper和actor
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polydata);
        
        trajectory_actor_->SetMapper(mapper);
        trajectory_actor_->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色轨迹
        trajectory_actor_->GetProperty()->SetLineWidth(3.0);
        
        // 刷新渲染
        render_window_->Render();
        
        qDebug() << "轨迹可视化更新:" << path_msg->poses.size() << "个位姿";
        
    } catch (const std::exception& e) {
        qWarning() << "轨迹可视化更新失败:" << e.what();
    }
}

void VisualizationWidget::updateKeyframesVisualization()
{
    // 简化实现 - 实际项目中需要关键帧可视化
}

void VisualizationWidget::updateLoopClosuresVisualization(const visualization_msgs::msg::MarkerArray::SharedPtr& markers_msg)
{
    // 简化实现 - 实际项目中需要回环可视化
    Q_UNUSED(markers_msg)
}

pcl::PointCloud<pcl::PointXYZI>::Ptr VisualizationWidget::downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // 简单的随机降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (input_cloud->empty()) return downsampled;
    
    size_t step = input_cloud->size() / settings_.max_points_display + 1;
    
    for (size_t i = 0; i < input_cloud->size(); i += step) {
        downsampled->push_back((*input_cloud)[i]);
        if (downsampled->size() >= static_cast<size_t>(settings_.max_points_display)) {
            break;
        }
    }
    
    return downsampled;
}

// VTK相关的颜色映射函数已暂时移除

void VisualizationWidget::setupCoordinateAxes()
{
    if (!vtk_available_ || !renderer_) return;
    
    try {
        // 创建坐标轴
        auto axes = vtkSmartPointer<vtkAxesActor>::New();
        axes->SetTotalLength(5.0, 5.0, 5.0);
        axes->SetCylinderRadius(0.05);
        axes->SetConeRadius(0.1);
        axes->SetSphereRadius(0.1);
        
        // 添加到渲染器
        renderer_->AddActor(axes);
        
        // 创建OrientationMarker
        auto orientationMarker = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        orientationMarker->SetOrientationMarker(axes);
        if (vtk_native_widget_) {
            orientationMarker->SetInteractor(vtk_native_widget_->renderWindow()->GetInteractor());
            orientationMarker->SetViewport(0.0, 0.0, 0.25, 0.25);
            orientationMarker->SetEnabled(1);
        }
        
        qDebug() << "坐标轴设置完成";
        
    } catch (const std::exception& e) {
        qWarning() << "坐标轴设置失败:" << e.what();
    }
}

void VisualizationWidget::setupLighting()
{
    if (!vtk_available_ || !renderer_) return;
    
    try {
        // 移除默认光源
        renderer_->RemoveAllLights();
        
        // 添加头灯
        auto headLight = vtkSmartPointer<vtkLight>::New();
        headLight->SetLightTypeToHeadlight();
        headLight->SetIntensity(0.8);
        headLight->SetColor(1.0, 1.0, 1.0);
        renderer_->AddLight(headLight);
        
        // 添加环境光
        auto ambientLight = vtkSmartPointer<vtkLight>::New();
        ambientLight->SetLightTypeToSceneLight();
        ambientLight->SetPosition(0, 0, 20);
        ambientLight->SetIntensity(0.3);
        renderer_->AddLight(ambientLight);
        
        // 设置环境光强度
        renderer_->SetAmbient(0.2, 0.2, 0.2);
        
        qDebug() << "光照设置完成";
        
    } catch (const std::exception& e) {
        qWarning() << "光照设置失败:" << e.what();
    }
}

void VisualizationWidget::resetView()
{
    if (!vtk_available_ || !renderer_ || !render_window_) {
        qDebug() << "重置视角（VTK未初始化）";
        return;
    }
    
    try {
        // 重置相机位置
        auto camera = renderer_->GetActiveCamera();
        camera->SetPosition(0, 0, 50);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
        
        // 自动调整视角以适应场景
        renderer_->ResetCamera();
        
        // 刷新渲染
        render_window_->Render();
        
        qDebug() << "视角已重置";
        
    } catch (const std::exception& e) {
        qWarning() << "重置视角失败:" << e.what();
    }
}

void VisualizationWidget::centerView()
{
    resetView();
}

void VisualizationWidget::clearAllData()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_slam_data_ = SLAMData();
    if (current_point_cloud_) {
        current_point_cloud_->clear();
    }
    current_trajectory_.clear();
}

// 槽函数实现
void VisualizationWidget::onPointCloudToggled(bool enabled)
{
    settings_.show_point_cloud = enabled;
    qDebug() << "点云显示切换:" << enabled;
}

void VisualizationWidget::onTrajectoryToggled(bool enabled)
{
    settings_.show_trajectory = enabled;
    qDebug() << "轨迹显示切换:" << enabled;
}

void VisualizationWidget::onKeyframesToggled(bool enabled)
{
    settings_.show_keyframes = enabled;
    qDebug() << "关键帧显示切换:" << enabled;
}

void VisualizationWidget::onLoopClosuresToggled(bool enabled)
{
    settings_.show_loop_closures = enabled;
    qDebug() << "回环显示切换:" << enabled;
}

void VisualizationWidget::onColorModeChanged(int mode)
{
    settings_.point_cloud_color_mode = static_cast<VisualizationSettings::ColorMode>(mode);
    data_updated_ = true; // 触发重新渲染
}

void VisualizationWidget::onPointSizeChanged(double size)
{
    settings_.point_size = size;
    qDebug() << "点大小调整:" << size;
}

void VisualizationWidget::onMaxPointsChanged(int max_points)
{
    settings_.max_points_display = max_points;
    data_updated_ = true; // 触发重新处理点云
}

void VisualizationWidget::onResetView()
{
    resetView();
}

void VisualizationWidget::onBackgroundColorChanged()
{
    // 简化实现：暂时跳过背景颜色更改
    qDebug() << "背景颜色更改（功能待实现）";
}

// 颜色映射函数实现
std::array<unsigned char, 3> VisualizationWidget::getPointColor(const pcl::PointXYZI& point)
{
    std::array<unsigned char, 3> color = {255, 255, 255}; // 默认白色
    
    switch (settings_.point_cloud_color_mode) {
        case VisualizationSettings::HEIGHT_COLOR:
            // 高度着色：蓝色(低) -> 绿色(中) -> 红色(高)
            {
                float normalized_z = std::max(0.0f, std::min(1.0f, (point.z + 10.0f) / 20.0f));
                if (normalized_z < 0.5f) {
                    // 蓝色到绿色
                    color[0] = 0;
                    color[1] = static_cast<unsigned char>(255 * normalized_z * 2);
                    color[2] = static_cast<unsigned char>(255 * (1 - normalized_z * 2));
                } else {
                    // 绿色到红色
                    color[0] = static_cast<unsigned char>(255 * (normalized_z - 0.5f) * 2);
                    color[1] = static_cast<unsigned char>(255 * (1 - (normalized_z - 0.5f) * 2));
                    color[2] = 0;
                }
            }
            break;
            
        case VisualizationSettings::INTENSITY_COLOR:
            // 强度着色：黑色(低强度) -> 白色(高强度)
            {
                unsigned char intensity = static_cast<unsigned char>(std::max(0.0f, std::min(255.0f, point.intensity)));
                color[0] = intensity;
                color[1] = intensity;
                color[2] = intensity;
            }
            break;
            
        case VisualizationSettings::TIME_COLOR:
            // 时间着色：使用彩虹色谱
            {
                static auto start_time = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                float time_diff = std::chrono::duration<float>(now - start_time).count();
                float hue = fmod(time_diff * 30.0f, 360.0f); // 30度/秒的色相变化
                
                // HSV to RGB 转换
                float h = hue / 60.0f;
                float s = 1.0f;
                float v = 1.0f;
                
                int hi = static_cast<int>(floor(h)) % 6;
                float f = h - floor(h);
                float p = v * (1 - s);
                float q = v * (1 - s * f);
                float t = v * (1 - s * (1 - f));
                
                float r, g, b;
                switch (hi) {
                    case 0: r = v; g = t; b = p; break;
                    case 1: r = q; g = v; b = p; break;
                    case 2: r = p; g = v; b = t; break;
                    case 3: r = p; g = q; b = v; break;
                    case 4: r = t; g = p; b = v; break;
                    case 5: r = v; g = p; b = q; break;
                    default: r = g = b = 1.0f; break;
                }
                
                color[0] = static_cast<unsigned char>(r * 255);
                color[1] = static_cast<unsigned char>(g * 255);
                color[2] = static_cast<unsigned char>(b * 255);
            }
            break;
            
        case VisualizationSettings::NORMAL_COLOR:
            // 法向量着色（简化版本）
            {
                float nx = std::abs(point.x);
                float ny = std::abs(point.y);
                float nz = std::abs(point.z);
                float norm = std::sqrt(nx*nx + ny*ny + nz*nz);
                if (norm > 0) {
                    color[0] = static_cast<unsigned char>(255 * nx / norm);
                    color[1] = static_cast<unsigned char>(255 * ny / norm);
                    color[2] = static_cast<unsigned char>(255 * nz / norm);
                }
            }
            break;
            
        case VisualizationSettings::CUSTOM_COLOR:
        default:
            // 自定义颜色：浅蓝色
            color[0] = 100;
            color[1] = 150;
            color[2] = 255;
            break;
    }
    
    return color;
}

// MOC文件由CMake自动处理