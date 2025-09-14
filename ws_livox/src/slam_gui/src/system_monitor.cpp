#include "slam_gui/system_monitor.h"
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QProcess>
#include <chrono>
#include <fstream>
#include <sstream>

SystemMonitor::SystemMonitor(QObject* parent)
    : QObject(parent)
    , monitoring_active_(false)
    , update_interval_(1000)
{
    update_timer_ = new QTimer(this);
    process_timer_ = new QTimer(this);
    
    connect(update_timer_, &QTimer::timeout, this, &SystemMonitor::updateMetrics);
    connect(process_timer_, &QTimer::timeout, this, &SystemMonitor::checkProcesses);
    
    // 添加要监控的进程
    monitored_processes_ << "lio_node" << "pgo_node" << "hba_node" << "localizer_node" << "livox_ros_driver2_node";
    
    // 初始化计数器
    last_network_counters_.timestamp = std::chrono::steady_clock::now();
    last_cpu_counters_.timestamp = std::chrono::steady_clock::now();
}

SystemMonitor::~SystemMonitor()
{
    stopMonitoring();
}

void SystemMonitor::startMonitoring()
{
    if (monitoring_active_) return;
    
    monitoring_active_ = true;
    
    // 启动定时器
    update_timer_->start(update_interval_);
    process_timer_->start(2000); // 2秒检查一次进程
    
    // 立即更新一次
    updateMetrics();
    checkProcesses();
}

void SystemMonitor::stopMonitoring()
{
    monitoring_active_ = false;
    
    if (update_timer_) update_timer_->stop();
    if (process_timer_) process_timer_->stop();
}

SystemMetrics SystemMonitor::getCurrentMetrics() const
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    return current_metrics_;
}

void SystemMonitor::addProcessToMonitor(const QString& process_name)
{
    if (!monitored_processes_.contains(process_name)) {
        monitored_processes_.append(process_name);
    }
}

void SystemMonitor::removeProcessFromMonitor(const QString& process_name)
{
    monitored_processes_.removeAll(process_name);
}

void SystemMonitor::setUpdateInterval(int interval_ms)
{
    update_interval_ = interval_ms;
    if (monitoring_active_) {
        update_timer_->start(update_interval_);
    }
}

void SystemMonitor::updateMetrics()
{
    if (!monitoring_active_) return;
    
    updateSystemMetrics();
    updateProcessMetrics();
    checkSystemAlerts();
    
    emit metricsUpdated(current_metrics_);
}

void SystemMonitor::updateSystemMetrics()
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    // 更新CPU使用率
    current_metrics_.total_cpu_usage = getCPUUsage();
    
    // 更新内存使用率
    current_metrics_.total_memory_usage = getMemoryUsage();
    
    // 更新磁盘使用率
    current_metrics_.disk_usage = getDiskUsage();
    
    // 更新网络使用率
    getNetworkUsage(current_metrics_.network_tx_rate, current_metrics_.network_rx_rate);
    
    // 更新历史数据
    current_metrics_.cpu_history.push_back(current_metrics_.total_cpu_usage);
    if (current_metrics_.cpu_history.size() > HISTORY_SIZE) {
        current_metrics_.cpu_history.erase(current_metrics_.cpu_history.begin());
    }
    
    current_metrics_.memory_history.push_back(current_metrics_.total_memory_usage);
    if (current_metrics_.memory_history.size() > HISTORY_SIZE) {
        current_metrics_.memory_history.erase(current_metrics_.memory_history.begin());
    }
}

void SystemMonitor::updateProcessMetrics()
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    
    for (const QString& process_name : monitored_processes_) {
        ProcessInfo info = getProcessInfo(process_name);
        
        bool was_running = current_metrics_.processes.find(process_name) != current_metrics_.processes.end() && 
                          current_metrics_.processes[process_name].is_running;
        
        current_metrics_.processes[process_name] = info;
        
        // 如果状态改变，发送信号
        if (was_running != info.is_running) {
            emit processStatusChanged(process_name, info.is_running);
        }
    }
}

void SystemMonitor::checkProcesses()
{
    updateProcessMetrics();
}

void SystemMonitor::checkSystemAlerts()
{
    // 检查CPU使用率
    if (current_metrics_.total_cpu_usage > CPU_ALERT_THRESHOLD) {
        emit systemAlertTriggered(QString("CPU使用率过高: %1%").arg(current_metrics_.total_cpu_usage, 0, 'f', 1));
    }
    
    // 检查内存使用率  
    if (current_metrics_.total_memory_usage > MEMORY_ALERT_THRESHOLD) {
        emit systemAlertTriggered(QString("内存使用率过高: %1%").arg(current_metrics_.total_memory_usage, 0, 'f', 1));
    }
    
    // 检查磁盘使用率
    if (current_metrics_.disk_usage > DISK_ALERT_THRESHOLD) {
        emit systemAlertTriggered(QString("磁盘使用率过高: %1%").arg(current_metrics_.disk_usage, 0, 'f', 1));
    }
}

ProcessInfo SystemMonitor::getProcessInfo(const QString& process_name)
{
    ProcessInfo info;
    info.name = process_name;
    
    // 使用pgrep查找进程
    QProcess pgrep;
    pgrep.start("pgrep", QStringList() << "-f" << process_name);
    pgrep.waitForFinished(1000);
    
    if (pgrep.exitCode() == 0) {
        QString output = pgrep.readAllStandardOutput().trimmed();
        QStringList pids = output.split('\n', Qt::SkipEmptyParts);
        
        if (!pids.isEmpty()) {
            info.pid = pids.first().toInt();
            info.is_running = true;
            info.status = "运行中";
            
            // 获取CPU和内存使用率
            QProcess ps;
            ps.start("ps", QStringList() << "-p" << QString::number(info.pid) << "-o" << "pcpu,pmem" << "--no-headers");
            ps.waitForFinished(1000);
            
            if (ps.exitCode() == 0) {
                QString ps_output = ps.readAllStandardOutput().trimmed();
                QStringList values = ps_output.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
                
                if (values.size() >= 2) {
                    info.cpu_usage = values[0].toDouble();
                    info.memory_usage = values[1].toDouble();
                }
            }
        }
    } else {
        info.is_running = false;
        info.status = "未运行";
        info.cpu_usage = 0.0;
        info.memory_usage = 0.0;
    }
    
    return info;
}

double SystemMonitor::getCPUUsage()
{
    static unsigned long long last_idle = 0, last_total = 0;
    
    std::ifstream file("/proc/stat");
    if (!file.is_open()) return 0.0;
    
    std::string line;
    std::getline(file, line);
    
    std::istringstream ss(line);
    std::string cpu;
    ss >> cpu;
    
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
    ss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    
    unsigned long long total = user + nice + system + idle + iowait + irq + softirq + steal;
    unsigned long long current_idle = idle + iowait;
    
    double cpu_usage = 0.0;
    if (last_total != 0) {
        unsigned long long total_diff = total - last_total;
        unsigned long long idle_diff = current_idle - last_idle;
        
        if (total_diff > 0) {
            cpu_usage = 100.0 * (total_diff - idle_diff) / total_diff;
        }
    }
    
    last_idle = current_idle;
    last_total = total;
    
    return cpu_usage;
}

double SystemMonitor::getMemoryUsage()
{
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) return 0.0;
    
    std::string line;
    unsigned long long mem_total = 0, mem_available = 0;
    
    while (std::getline(file, line)) {
        if (line.find("MemTotal:") == 0) {
            std::istringstream ss(line);
            std::string label;
            ss >> label >> mem_total;
        } else if (line.find("MemAvailable:") == 0) {
            std::istringstream ss(line);
            std::string label;
            ss >> label >> mem_available;
            break;
        }
    }
    
    if (mem_total > 0) {
        current_metrics_.available_memory_gb = mem_available / 1024.0 / 1024.0;
        return 100.0 * (mem_total - mem_available) / mem_total;
    }
    
    return 0.0;
}

double SystemMonitor::getDiskUsage()
{
    QProcess df;
    df.start("df", QStringList() << "-h" << "/");
    df.waitForFinished(2000);
    
    if (df.exitCode() == 0) {
        QString output = df.readAllStandardOutput();
        QStringList lines = output.split('\n', Qt::SkipEmptyParts);
        
        if (lines.size() >= 2) {
            QStringList fields = lines[1].split(QRegExp("\\s+"), Qt::SkipEmptyParts);
            if (fields.size() >= 5) {
                QString usage_str = fields[4];
                if (usage_str.endsWith('%')) {
                    usage_str.chop(1);
                    return usage_str.toDouble();
                }
            }
        }
    }
    
    return 0.0;
}

void SystemMonitor::getNetworkUsage(double& tx_rate, double& rx_rate)
{
    tx_rate = 0.0;
    rx_rate = 0.0;
    
    std::ifstream file("/proc/net/dev");
    if (!file.is_open()) return;
    
    std::string line;
    // 跳过头两行
    std::getline(file, line);
    std::getline(file, line);
    
    unsigned long long total_rx = 0, total_tx = 0;
    
    while (std::getline(file, line)) {
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        
        std::string interface = line.substr(0, colon);
        interface.erase(0, interface.find_first_not_of(" \t"));
        
        // 跳过回环接口
        if (interface == "lo") continue;
        
        std::string data = line.substr(colon + 1);
        std::istringstream ss(data);
        
        unsigned long long rx_bytes, rx_packets, rx_errs, rx_drop, rx_fifo, rx_frame, rx_compressed, rx_multicast;
        unsigned long long tx_bytes, tx_packets, tx_errs, tx_drop, tx_fifo, tx_colls, tx_carrier, tx_compressed;
        
        ss >> rx_bytes >> rx_packets >> rx_errs >> rx_drop >> rx_fifo >> rx_frame >> rx_compressed >> rx_multicast
           >> tx_bytes >> tx_packets >> tx_errs >> tx_drop >> tx_fifo >> tx_colls >> tx_carrier >> tx_compressed;
        
        total_rx += rx_bytes;
        total_tx += tx_bytes;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_network_counters_.timestamp).count();
    
    if (time_diff > 0 && last_network_counters_.rx_bytes > 0) {
        rx_rate = (total_rx - last_network_counters_.rx_bytes) * 1000.0 / time_diff; // bytes/sec
        tx_rate = (total_tx - last_network_counters_.tx_bytes) * 1000.0 / time_diff;
    }
    
    last_network_counters_.rx_bytes = total_rx;
    last_network_counters_.tx_bytes = total_tx;
    last_network_counters_.timestamp = now;
}

// MOC文件由CMake自动处理