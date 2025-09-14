#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include <QStandardPaths>
#include <QMessageBox>
#include <QSplashScreen>
#include <QPixmap>
#include <QTimer>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "slam_gui/main_window.h"

void setupApplication(QApplication& app)
{
    // 设置应用程序信息
    app.setApplicationName("SLAM GUI");
    app.setApplicationVersion("1.0.0");
    app.setApplicationDisplayName("SLAM可视化控制中心");
    app.setOrganizationName("SLAM Team");
    app.setOrganizationDomain("slam.team");
    
    // 设置应用程序图标
    // app.setWindowIcon(QIcon(":/icons/slam_gui.png"));
    
    // 设置样式
    app.setStyle(QStyleFactory::create("Fusion"));
    
    // 设置深色主题
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    
    app.setPalette(darkPalette);
    
    // 设置样式表
    QString styleSheet = R"(
        QMainWindow {
            background-color: #353535;
        }
        
        QMenuBar {
            background-color: #2b2b2b;
            color: white;
            border-bottom: 1px solid #404040;
        }
        
        QMenuBar::item {
            background-color: transparent;
            padding: 4px 8px;
        }
        
        QMenuBar::item:selected {
            background-color: #404040;
        }
        
        QMenu {
            background-color: #2b2b2b;
            color: white;
            border: 1px solid #404040;
        }
        
        QMenu::item:selected {
            background-color: #404040;
        }
        
        QToolBar {
            background-color: #2b2b2b;
            border: 1px solid #404040;
            spacing: 2px;
        }
        
        QStatusBar {
            background-color: #2b2b2b;
            color: white;
            border-top: 1px solid #404040;
        }
        
        QPushButton {
            background-color: #404040;
            color: white;
            border: 1px solid #555555;
            padding: 6px 12px;
            border-radius: 3px;
        }
        
        QPushButton:hover {
            background-color: #4a4a4a;
        }
        
        QPushButton:pressed {
            background-color: #2a2a2a;
        }
        
        QPushButton:disabled {
            background-color: #2a2a2a;
            color: #666666;
        }
        
        QGroupBox {
            font-weight: bold;
            border: 2px solid #404040;
            border-radius: 5px;
            margin: 3px;
            padding-top: 10px;
        }
        
        QGroupBox::title {
            subcontrol-origin: margin;
            subcontrol-position: top center;
            padding: 0 5px;
            color: white;
        }
        
        QTabWidget::pane {
            border: 1px solid #404040;
            background-color: #353535;
        }
        
        QTabBar::tab {
            background-color: #2b2b2b;
            color: white;
            padding: 6px 12px;
            border: 1px solid #404040;
            border-bottom: none;
        }
        
        QTabBar::tab:selected {
            background-color: #353535;
        }
        
        QTabBar::tab:hover {
            background-color: #404040;
        }
        
        QTextEdit {
            background-color: #1e1e1e;
            color: white;
            border: 1px solid #404040;
            selection-background-color: #2a82da;
        }
        
        QTreeWidget, QTableWidget {
            background-color: #1e1e1e;
            color: white;
            border: 1px solid #404040;
            alternate-background-color: #2a2a2a;
        }
        
        QHeaderView::section {
            background-color: #2b2b2b;
            color: white;
            padding: 4px;
            border: 1px solid #404040;
        }
        
        QScrollBar:vertical {
            background-color: #2b2b2b;
            width: 12px;
            border-radius: 6px;
        }
        
        QScrollBar::handle:vertical {
            background-color: #555555;
            border-radius: 6px;
            min-height: 20px;
        }
        
        QScrollBar::handle:vertical:hover {
            background-color: #666666;
        }
        
        QProgressBar {
            border: 1px solid #404040;
            border-radius: 3px;
            text-align: center;
            background-color: #1e1e1e;
        }
        
        QProgressBar::chunk {
            background-color: #2a82da;
            border-radius: 2px;
        }
    )";
    
    app.setStyleSheet(styleSheet);
}

bool checkROS2Environment()
{
    // 检查ROS2环境
    const char* ros_distro = std::getenv("ROS_DISTRO");
    if (!ros_distro) {
        QMessageBox::critical(nullptr, "ROS2环境错误", 
            "未检测到ROS2环境变量。\n"
            "请确保已正确安装ROS2并source了setup脚本。\n\n"
            "例如: source /opt/ros/humble/setup.bash");
        return false;
    }
    
    std::cout << "检测到ROS2发行版: " << ros_distro << std::endl;
    
    // 检查是否在ROS2工作空间中
    const char* ament_prefix_path = std::getenv("AMENT_PREFIX_PATH");
    if (!ament_prefix_path) {
        QMessageBox::warning(nullptr, "工作空间警告",
            "未检测到AMENT_PREFIX_PATH环境变量。\n"
            "确保已source了工作空间的setup脚本。");
    }
    
    return true;
}

void showSplashScreen(QApplication& app)
{
    // 创建启动画面
    QPixmap splash_pixmap(400, 300);
    splash_pixmap.fill(QColor(53, 53, 53));
    
    QSplashScreen splash(splash_pixmap);
    splash.setStyleSheet("color: white; font-size: 14px;");
    splash.show();
    
    splash.showMessage("正在初始化SLAM GUI...", Qt::AlignBottom | Qt::AlignCenter);
    app.processEvents();
    
    // 模拟初始化过程
    QTimer::singleShot(1000, [&splash, &app]() {
        splash.showMessage("正在连接ROS2...", Qt::AlignBottom | Qt::AlignCenter);
        app.processEvents();
    });
    
    QTimer::singleShot(2000, [&splash, &app]() {
        splash.showMessage("正在加载配置...", Qt::AlignBottom | Qt::AlignCenter);
        app.processEvents();
    });
    
    QTimer::singleShot(3000, [&splash]() {
        splash.close();
    });
    
    // 等待3秒
    QTimer timer;
    timer.setSingleShot(true);
    QEventLoop loop;
    QObject::connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    timer.start(3000);
    loop.exec();
}

int main(int argc, char *argv[])
{
    // 创建Qt应用程序
    QApplication app(argc, argv);
    
    // 设置应用程序属性
    setupApplication(app);
    
    std::cout << "启动SLAM可视化控制中心..." << std::endl;
    
    // 检查ROS2环境
    if (!checkROS2Environment()) {
        return -1;
    }
    
    try {
        // 初始化ROS2
        rclcpp::init(argc, argv);
        std::cout << "ROS2初始化成功" << std::endl;
        
        // 显示启动画面
        showSplashScreen(app);
        
        // 创建主窗口
        MainWindow main_window;
        main_window.show();
        
        std::cout << "主窗口已显示" << std::endl;
        
        // 运行应用程序
        int result = app.exec();
        
        // 清理ROS2
        rclcpp::shutdown();
        std::cout << "应用程序正常退出" << std::endl;
        
        return result;
        
    } catch (const std::exception& e) {
        std::cerr << "应用程序异常: " << e.what() << std::endl;
        
        QMessageBox::critical(nullptr, "致命错误", 
            QString("应用程序遇到致命错误:\n%1\n\n程序将退出。").arg(e.what()));
        
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        
        return -1;
    } catch (...) {
        std::cerr << "未知异常" << std::endl;
        
        QMessageBox::critical(nullptr, "未知错误", 
            "应用程序遇到未知错误，程序将退出。");
        
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        
        return -1;
    }
}