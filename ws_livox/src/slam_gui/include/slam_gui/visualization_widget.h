#ifndef SLAM_GUI_VISUALIZATION_WIDGET_H
#define SLAM_GUI_VISUALIZATION_WIDGET_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QSplitter>
#include <QTimer>
#include <memory>
#include <mutex>

// VTK includes for 3D visualization
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkLight.h>

// Qt-VTK integration
#include <QVTKOpenGLNativeWidget.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "ros2_interface.h"

struct VisualizationSettings
{
    // 点云显示设置
    bool show_point_cloud = true;
    bool show_trajectory = true;
    bool show_keyframes = false;
    bool show_loop_closures = false;
    bool show_coordinate_axes = true;
    bool show_ground_plane = false;
    
    // 颜色设置
    enum ColorMode {
        HEIGHT_COLOR,
        INTENSITY_COLOR,
        TIME_COLOR,
        NORMAL_COLOR,
        CUSTOM_COLOR
    };
    ColorMode point_cloud_color_mode = HEIGHT_COLOR;
    
    // 点云渲染设置
    double point_size = 2.0;
    double trajectory_width = 0.1;
    int max_points_display = 500000; // 最大显示点数
    double downsample_resolution = 0.1; // 降采样分辨率
    
    // 视图控制
    bool auto_center = true;
    bool follow_trajectory = false;
    double view_distance = 50.0;
    
    // 背景和环境
    double background_color[3] = {0.1, 0.1, 0.2}; // 深蓝色背景
    bool use_gradient_background = true;
    
    // 性能设置
    int max_fps = 30;
    bool enable_point_picking = true;
    bool enable_measurement = false;
};

class VisualizationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit VisualizationWidget(QWidget* parent = nullptr);
    ~VisualizationWidget();

    void updateSLAMData(const SLAMData& data);
    void setVisualizationSettings(const VisualizationSettings& settings);
    VisualizationSettings getVisualizationSettings() const;
    
    // 视图控制
    void resetView();
    void centerView();
    void fitToData();
    void setViewMode(const QString& mode); // "top", "side", "front", "iso"
    
    // 数据操作
    void clearAllData();
    void saveScreenshot(const QString& filename);
    void exportPointCloud(const QString& filename);
    
    // 交互功能
    void enablePointPicking(bool enable);
    void enableMeasurement(bool enable);

signals:
    void pointPicked(double x, double y, double z);
    void measurementCompleted(double distance);
    void viewChanged();

public slots:
    void onPointCloudToggled(bool enabled);
    void onTrajectoryToggled(bool enabled);
    void onKeyframesToggled(bool enabled);
    void onLoopClosuresToggled(bool enabled);
    void onColorModeChanged(int mode);
    void onPointSizeChanged(double size);
    void onMaxPointsChanged(int max_points);
    void onBackgroundColorChanged();
    void onResetView();

private slots:
    void updateVisualization();

private:
    void setupUI();
    void setupVTK();
    void setupControlPanel();
    void connectSignals();
    
    // VTK数据更新
    void updatePointCloudVisualization(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);
    void updateTrajectoryVisualization(const nav_msgs::msg::Path::SharedPtr& path_msg);
    void updateKeyframesVisualization();
    void updateLoopClosuresVisualization(const visualization_msgs::msg::MarkerArray::SharedPtr& markers_msg);
    
    // 颜色映射
    void applyColorMapping(vtkSmartPointer<vtkPolyData> polydata, 
                          VisualizationSettings::ColorMode mode);
    void applyHeightColorMapping(vtkSmartPointer<vtkPolyData> polydata);
    void applyIntensityColorMapping(vtkSmartPointer<vtkPolyData> polydata);
    void applyTimeColorMapping(vtkSmartPointer<vtkPolyData> polydata);
    
    // VTK辅助函数
    vtkSmartPointer<vtkLookupTable> createColorLUT(VisualizationSettings::ColorMode mode);
    void updateScalarBar(const QString& title, vtkSmartPointer<vtkLookupTable> lut);
    
    // 数据处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud);
    
    // 颜色映射辅助函数
    std::array<unsigned char, 3> getPointColor(const pcl::PointXYZI& point);
    
    // 视图辅助
    void updateCameraPosition();
    void setupLighting();
    void setupCoordinateAxes();

private:
    // UI组件
    QVBoxLayout* main_layout_;
    QHBoxLayout* top_layout_;
    QSplitter* splitter_;
    
    // VTK组件
    QWidget* vtk_widget_;  // 通用widget指针，可能是QVTKOpenGLNativeWidget或普通QWidget
    QVTKOpenGLNativeWidget* vtk_native_widget_;  // VTK专用widget
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window_;
    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactor_style_;
    bool vtk_available_;  // VTK是否可用的标志
    
    // VTK Actors
    vtkSmartPointer<vtkActor> point_cloud_actor_;
    vtkSmartPointer<vtkActor> trajectory_actor_;
    vtkSmartPointer<vtkActor> keyframes_actor_;
    vtkSmartPointer<vtkActor> loop_closures_actor_;
    vtkSmartPointer<vtkAxesActor> axes_actor_;
    vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;
    vtkSmartPointer<vtkScalarBarActor> scalar_bar_actor_;
    
    // 控制面板
    QGroupBox* control_panel_;
    QCheckBox* show_point_cloud_cb_;
    QCheckBox* show_trajectory_cb_;
    QCheckBox* show_keyframes_cb_;
    QCheckBox* show_loop_closures_cb_;
    QComboBox* color_mode_combo_;
    QDoubleSpinBox* point_size_spin_;
    QSpinBox* max_points_spin_;
    QPushButton* reset_view_btn_;
    QPushButton* center_view_btn_;
    QPushButton* screenshot_btn_;
    
    // 数据存储
    SLAMData current_slam_data_;
    VisualizationSettings settings_;
    
    // 性能和状态
    QTimer* update_timer_;
    bool data_updated_;
    
    // 点云处理
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud_;
    std::vector<geometry_msgs::msg::PoseStamped> current_trajectory_;
    
    mutable std::mutex data_mutex_;
};

#endif // SLAM_GUI_VISUALIZATION_WIDGET_H