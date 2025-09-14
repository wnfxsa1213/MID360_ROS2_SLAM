#include "slam_gui/log_viewer.h"
#include <QDateTime>
#include <QTextStream>
#include <QMessageBox>
#include <QScrollBar>

LogViewer::LogViewer(QWidget* parent)
    : QWidget(parent)
    , main_layout_(nullptr)
    , control_layout_(nullptr)
    , splitter_(nullptr)
    , control_panel_(nullptr)
    , log_display_(nullptr)
    , max_log_entries_(DEFAULT_MAX_ENTRIES)
    , auto_scroll_enabled_(true)
    , word_wrap_enabled_(true)
    , min_log_level_(LogEntry::Debug)
    , needs_update_(false)
{
    setupUI();
    connectSignals();
    
    // 初始化更新定时器
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &LogViewer::updateDisplay);
    update_timer_->start(UPDATE_INTERVAL_MS);
    
    // 添加欢迎消息
    addLogMessage(LogEntry::Info, "日志查看器已初始化", "LogViewer");
}

LogViewer::~LogViewer()
{
    if (update_timer_) {
        update_timer_->stop();
    }
}

void LogViewer::setupUI()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(2, 2, 2, 2);
    
    setupControlPanel();
    setupLogDisplay();
    
    setLayout(main_layout_);
}

void LogViewer::setupControlPanel()
{
    control_panel_ = new QGroupBox("日志控制");
    control_panel_->setMaximumHeight(80);
    
    control_layout_ = new QHBoxLayout(control_panel_);
    
    // 日志级别过滤
    control_layout_->addWidget(new QLabel("级别:"));
    level_filter_combo_ = new QComboBox();
    level_filter_combo_->addItems({"全部", "调试", "信息", "警告", "错误", "严重"});
    level_filter_combo_->setCurrentIndex(1); // 默认显示INFO及以上级别
    level_filter_combo_->setMaximumWidth(80);
    control_layout_->addWidget(level_filter_combo_);
    
    // 搜索框
    control_layout_->addWidget(new QLabel("搜索:"));
    search_edit_ = new QLineEdit();
    search_edit_->setPlaceholderText("输入搜索关键词...");
    search_edit_->setMaximumWidth(150);
    control_layout_->addWidget(search_edit_);
    
    // 控制按钮
    clear_btn_ = new QPushButton("清空");
    clear_btn_->setMaximumWidth(50);
    control_layout_->addWidget(clear_btn_);
    
    save_btn_ = new QPushButton("保存");
    save_btn_->setMaximumWidth(50);
    control_layout_->addWidget(save_btn_);
    
    // 选项
    auto_scroll_cb_ = new QCheckBox("自动滚动");
    auto_scroll_cb_->setChecked(auto_scroll_enabled_);
    control_layout_->addWidget(auto_scroll_cb_);
    
    word_wrap_cb_ = new QCheckBox("自动换行");
    word_wrap_cb_->setChecked(word_wrap_enabled_);
    control_layout_->addWidget(word_wrap_cb_);
    
    control_layout_->addStretch();
    
    // 日志统计
    log_count_label_ = new QLabel("日志: 0/0");
    log_count_label_->setStyleSheet("color: #888;");
    control_layout_->addWidget(log_count_label_);
    
    main_layout_->addWidget(control_panel_);
}

void LogViewer::setupLogDisplay()
{
    log_display_ = new QTextEdit();
    log_display_->setReadOnly(true);
    log_display_->setFont(QFont("Consolas", 9));
    log_display_->setWordWrapMode(word_wrap_enabled_ ? QTextOption::WrapAtWordBoundaryOrAnywhere : QTextOption::NoWrap);
    
    // 设置样式
    log_display_->setStyleSheet(R"(
        QTextEdit {
            background-color: #1e1e1e;
            color: #ffffff;
            border: 1px solid #404040;
            selection-background-color: #264f78;
        }
    )");
    
    main_layout_->addWidget(log_display_);
}

void LogViewer::connectSignals()
{
    connect(level_filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LogViewer::onLevelFilterChanged);
    connect(search_edit_, &QLineEdit::textChanged, this, &LogViewer::onSearchTextChanged);
    connect(clear_btn_, &QPushButton::clicked, this, &LogViewer::onClearLogsClicked);
    connect(save_btn_, &QPushButton::clicked, this, &LogViewer::onSaveLogsClicked);
    connect(auto_scroll_cb_, &QCheckBox::toggled, this, &LogViewer::onAutoScrollToggled);
    connect(word_wrap_cb_, &QCheckBox::toggled, this, &LogViewer::onWordWrapToggled);
}

void LogViewer::addLogMessage(LogEntry::Level level, const QString& message, const QString& source)
{
    std::lock_guard<std::mutex> lock(logs_mutex_);
    
    LogEntry entry(level, source, message);
    log_entries_.push_back(entry);
    
    // 限制日志条目数量
    if (log_entries_.size() > static_cast<size_t>(max_log_entries_)) {
        log_entries_.pop_front();
    }
    
    // 检查是否是严重错误
    if (level == LogEntry::Critical) {
        emit criticalErrorOccurred(message);
    }
    
    needs_update_ = true;
    emit logEntryAdded(entry);
}

void LogViewer::addLogMessage(Level level, const QString& message, const QString& source)
{
    addLogMessage(static_cast<LogEntry::Level>(level), message, source);
}

void LogViewer::clearLogs()
{
    {
        std::lock_guard<std::mutex> lock(logs_mutex_);
        log_entries_.clear();
        filtered_entries_.clear();
    }
    
    log_display_->clear();
    updateLogCount();
    emit logCleared();
}

void LogViewer::saveLogsToFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "保存失败", "无法创建日志文件: " + filename);
        return;
    }
    
    QTextStream out(&file);
    out.setCodec("UTF-8");
    
    // 写入头部信息
    out << "# SLAM GUI 日志文件\n";
    out << "# 生成时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << "\n";
    out << "# 总日志数: " << log_entries_.size() << "\n";
    out << "# ============================================\n\n";
    
    {
        std::lock_guard<std::mutex> lock(logs_mutex_);
        for (const auto& entry : log_entries_) {
            out << entry.full_text << "\n";
        }
    }
    
    file.close();
    addLogMessage(LogEntry::Info, QString("日志已保存到: %1").arg(filename), "LogViewer");
}

void LogViewer::loadLogsFromFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "加载失败", "无法打开日志文件: " + filename);
        return;
    }
    
    QTextStream in(&file);
    in.setCodec("UTF-8");
    
    clearLogs();
    
    int loaded_count = 0;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#')) {
            continue;
        }
        
        // 简单解析日志行
        // 格式: [时间] [级别] 来源: 消息
        if (line.contains("] [") && line.contains("]: ")) {
            addLogMessage(LogEntry::Info, line, "LoadedFile");
            loaded_count++;
        }
    }
    
    file.close();
    addLogMessage(LogEntry::Info, QString("已从文件加载 %1 条日志").arg(loaded_count), "LogViewer");
}

void LogViewer::setMaxLogEntries(int max_entries)
{
    max_log_entries_ = max_entries;
    
    std::lock_guard<std::mutex> lock(logs_mutex_);
    while (log_entries_.size() > static_cast<size_t>(max_log_entries_)) {
        log_entries_.pop_front();
    }
    
    needs_update_ = true;
}

void LogViewer::setAutoScroll(bool enabled)
{
    auto_scroll_enabled_ = enabled;
    auto_scroll_cb_->setChecked(enabled);
}

void LogViewer::setLogLevel(LogEntry::Level min_level)
{
    min_log_level_ = min_level;
    level_filter_combo_->setCurrentIndex(static_cast<int>(min_level));
    applyFilters();
}

void LogViewer::updateDisplay()
{
    if (!needs_update_) return;
    
    applyFilters();
    
    log_display_->clear();
    
    // 只显示最近的一定数量日志以提高性能
    size_t start_index = 0;
    if (filtered_entries_.size() > MAX_DISPLAY_ENTRIES) {
        start_index = filtered_entries_.size() - MAX_DISPLAY_ENTRIES;
    }
    
    for (size_t i = start_index; i < filtered_entries_.size(); ++i) {
        const LogEntry& entry = filtered_entries_[i];
        
        // 设置文本格式
        QTextCharFormat format = getLogFormat(entry.level);
        
        QTextCursor cursor = log_display_->textCursor();
        cursor.movePosition(QTextCursor::End);
        cursor.insertText(entry.full_text + "\n", format);
    }
    
    // 自动滚动到底部
    if (auto_scroll_enabled_) {
        scrollToBottom();
    }
    
    updateLogCount();
    needs_update_ = false;
}

void LogViewer::applyFilters()
{
    std::lock_guard<std::mutex> lock(logs_mutex_);
    
    filtered_entries_.clear();
    
    for (const auto& entry : log_entries_) {
        if (matchesFilter(entry)) {
            filtered_entries_.push_back(entry);
        }
    }
}

bool LogViewer::matchesFilter(const LogEntry& entry) const
{
    // 级别过滤
    if (entry.level < min_log_level_) {
        return false;
    }
    
    // 搜索过滤
    if (!search_filter_.isEmpty()) {
        if (!entry.message.contains(search_filter_, Qt::CaseInsensitive) &&
            !entry.source.contains(search_filter_, Qt::CaseInsensitive)) {
            return false;
        }
    }
    
    return true;
}

void LogViewer::updateLogCount()
{
    log_count_label_->setText(QString("日志: %1/%2").arg(filtered_entries_.size()).arg(log_entries_.size()));
}

void LogViewer::scrollToBottom()
{
    QScrollBar* scrollbar = log_display_->verticalScrollBar();
    scrollbar->setValue(scrollbar->maximum());
}

QTextCharFormat LogViewer::getLogFormat(LogEntry::Level level) const
{
    QTextCharFormat format;
    format.setForeground(QBrush(LogEntry::levelToColor(level)));
    
    if (level == LogEntry::Error || level == LogEntry::Critical) {
        format.setFontWeight(QFont::Bold);
    }
    
    return format;
}

QString LogViewer::formatLogEntry(const LogEntry& entry) const
{
    return entry.full_text;
}

// 槽函数实现
void LogViewer::onFilterChanged()
{
    applyFilters();
    needs_update_ = true;
}

void LogViewer::onLevelFilterChanged(int level)
{
    if (level == 0) {
        min_log_level_ = LogEntry::Debug; // 显示全部
    } else {
        min_log_level_ = static_cast<LogEntry::Level>(level - 1);
    }
    onFilterChanged();
}

void LogViewer::onSearchTextChanged(const QString& text)
{
    search_filter_ = text;
    onFilterChanged();
}

void LogViewer::onClearLogsClicked()
{
    clearLogs();
}

void LogViewer::onSaveLogsClicked()
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString default_filename = QString("slam_gui_logs_%1.txt").arg(timestamp);
    
    QString filename = QFileDialog::getSaveFileName(this,
        "保存日志", default_filename, "Text Files (*.txt);;All Files (*)");
    
    if (!filename.isEmpty()) {
        saveLogsToFile(filename);
    }
}

void LogViewer::onAutoScrollToggled(bool enabled)
{
    auto_scroll_enabled_ = enabled;
    if (enabled) {
        scrollToBottom();
    }
}

void LogViewer::onWordWrapToggled(bool enabled)
{
    word_wrap_enabled_ = enabled;
    log_display_->setWordWrapMode(enabled ? QTextOption::WrapAtWordBoundaryOrAnywhere : QTextOption::NoWrap);
}

// MOC文件由CMake自动处理