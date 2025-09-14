#ifndef SLAM_GUI_SYSTEM_MONITOR_H
#define SLAM_GUI_SYSTEM_MONITOR_H

#include <QObject>
#include <QTimer>
#include <QProcess>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include <chrono>

struct ProcessInfo
{
    QString name;
    int pid = -1;
    double cpu_usage = 0.0;
    double memory_usage = 0.0;
    bool is_running = false;
    QString status;
};

struct SystemMetrics
{
    double total_cpu_usage = 0.0;
    double total_memory_usage = 0.0;
    double available_memory_gb = 0.0;
    double disk_usage = 0.0;
    double network_tx_rate = 0.0;
    double network_rx_rate = 0.0;
    
    std::map<QString, ProcessInfo> processes;
    std::vector<double> cpu_history;
    std::vector<double> memory_history;
};

class SystemMonitor : public QObject
{
    Q_OBJECT

public:
    explicit SystemMonitor(QObject* parent = nullptr);
    ~SystemMonitor();

    void startMonitoring();
    void stopMonitoring();
    bool isMonitoring() const { return monitoring_active_; }
    
    SystemMetrics getCurrentMetrics() const;
    
    void addProcessToMonitor(const QString& process_name);
    void removeProcessFromMonitor(const QString& process_name);
    
    void setUpdateInterval(int interval_ms);

signals:
    void metricsUpdated(const SystemMetrics& metrics);
    void processStatusChanged(const QString& process_name, bool is_running);
    void systemAlertTriggered(const QString& alert_message);

private slots:
    void updateMetrics();
    void checkProcesses();

private:
    void updateSystemMetrics();
    void updateProcessMetrics();
    void checkSystemAlerts();
    
    ProcessInfo getProcessInfo(const QString& process_name);
    double getCPUUsage();
    double getMemoryUsage();
    double getDiskUsage();
    void getNetworkUsage(double& tx_rate, double& rx_rate);

private:
    QTimer* update_timer_;
    QTimer* process_timer_;
    
    bool monitoring_active_;
    int update_interval_;
    
    mutable std::mutex metrics_mutex_;
    SystemMetrics current_metrics_;
    
    QStringList monitored_processes_;
    
    // 用于计算差值的历史数据
    struct NetworkCounters {
        unsigned long long tx_bytes = 0;
        unsigned long long rx_bytes = 0;
        std::chrono::steady_clock::time_point timestamp;
    } last_network_counters_;
    
    struct CPUCounters {
        unsigned long long idle = 0;
        unsigned long long total = 0;
        std::chrono::steady_clock::time_point timestamp;
    } last_cpu_counters_;
    
    // 告警阈值
    static constexpr double CPU_ALERT_THRESHOLD = 80.0;
    static constexpr double MEMORY_ALERT_THRESHOLD = 85.0;
    static constexpr double DISK_ALERT_THRESHOLD = 90.0;
    static constexpr size_t HISTORY_SIZE = 60; // 保留60个数据点
};

#endif // SLAM_GUI_SYSTEM_MONITOR_H