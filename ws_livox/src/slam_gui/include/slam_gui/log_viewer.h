#ifndef SLAM_GUI_LOG_VIEWER_H
#define SLAM_GUI_LOG_VIEWER_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLineEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QGroupBox>
#include <QSplitter>
#include <QTimer>
#include <QScrollBar>
#include <QTextCursor>
#include <QTextCharFormat>
#include <QDateTime>
#include <QFileDialog>
#include <QApplication>
#include <QFileDialog>
#include <memory>
#include <deque>
#include <mutex>

struct LogEntry
{
    enum Level {
        Debug = 0,
        Info = 1,
        Warning = 2,
        Error = 3,
        Critical = 4
    };
    
    Level level;
    QDateTime timestamp;
    QString source;
    QString message;
    QString full_text;
    
    LogEntry(Level lvl, const QString& src, const QString& msg)
        : level(lvl), timestamp(QDateTime::currentDateTime()), source(src), message(msg)
    {
        full_text = QString("[%1] [%2] %3: %4")
            .arg(timestamp.toString("hh:mm:ss.zzz"))
            .arg(levelToString(level))
            .arg(source)
            .arg(message);
    }
    
    static QString levelToString(Level level) {
        switch(level) {
            case Debug: return "DEBUG";
            case Info: return "INFO";
            case Warning: return "WARN";
            case Error: return "ERROR";
            case Critical: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
    
    static QColor levelToColor(Level level) {
        switch(level) {
            case Debug: return QColor(128, 128, 128);    // 灰色
            case Info: return QColor(0, 128, 0);         // 绿色
            case Warning: return QColor(255, 165, 0);    // 橙色
            case Error: return QColor(255, 0, 0);        // 红色
            case Critical: return QColor(139, 0, 0);     // 深红色
            default: return QColor(0, 0, 0);             // 黑色
        }
    }
};

class LogViewer : public QWidget
{
    Q_OBJECT

public:
    // 为了向后兼容，提供简化的Level枚举
    enum Level {
        Debug = LogEntry::Debug,
        Info = LogEntry::Info,
        Warning = LogEntry::Warning,
        Error = LogEntry::Error,
        Critical = LogEntry::Critical
    };

    explicit LogViewer(QWidget* parent = nullptr);
    ~LogViewer();

    void addLogMessage(LogEntry::Level level, const QString& message, const QString& source = "GUI");
    void addLogMessage(Level level, const QString& message, const QString& source = "GUI");
    
    void clearLogs();
    void saveLogsToFile(const QString& filename);
    void loadLogsFromFile(const QString& filename);
    
    void setMaxLogEntries(int max_entries);
    int getMaxLogEntries() const { return max_log_entries_; }
    
    void setAutoScroll(bool enabled);
    bool getAutoScroll() const { return auto_scroll_enabled_; }
    
    void setLogLevel(LogEntry::Level min_level);
    LogEntry::Level getLogLevel() const { return min_log_level_; }

signals:
    void logEntryAdded(const LogEntry& entry);
    void logCleared();
    void criticalErrorOccurred(const QString& message);

public slots:
    void onFilterChanged();
    void onLevelFilterChanged(int level);
    void onSearchTextChanged(const QString& text);
    void onClearLogsClicked();
    void onSaveLogsClicked();
    void onAutoScrollToggled(bool enabled);
    void onWordWrapToggled(bool enabled);

private slots:
    void updateDisplay();
    void scrollToBottom();

private:
    void setupUI();
    void setupControlPanel();
    void setupLogDisplay();
    void connectSignals();
    
    void appendLogEntry(const LogEntry& entry);
    void applyFilters();
    void updateLogCount();
    
    QString formatLogEntry(const LogEntry& entry) const;
    QTextCharFormat getLogFormat(LogEntry::Level level) const;
    
    bool matchesFilter(const LogEntry& entry) const;

private:
    // UI组件
    QVBoxLayout* main_layout_;
    QHBoxLayout* control_layout_;
    QSplitter* splitter_;
    
    // 控制面板
    QGroupBox* control_panel_;
    QComboBox* level_filter_combo_;
    QLineEdit* search_edit_;
    QPushButton* clear_btn_;
    QPushButton* save_btn_;
    QCheckBox* auto_scroll_cb_;
    QCheckBox* word_wrap_cb_;
    QLabel* log_count_label_;
    
    // 日志显示
    QTextEdit* log_display_;
    
    // 数据存储
    mutable std::mutex logs_mutex_;
    std::deque<LogEntry> log_entries_;
    std::vector<LogEntry> filtered_entries_;
    
    // 配置
    int max_log_entries_;
    bool auto_scroll_enabled_;
    bool word_wrap_enabled_;
    LogEntry::Level min_log_level_;
    QString search_filter_;
    
    // 更新控制
    QTimer* update_timer_;
    bool needs_update_;
    
    // 性能优化
    static constexpr int UPDATE_INTERVAL_MS = 100;
    static constexpr int MAX_DISPLAY_ENTRIES = 1000;
    static constexpr int DEFAULT_MAX_ENTRIES = 10000;
};

#endif // SLAM_GUI_LOG_VIEWER_H