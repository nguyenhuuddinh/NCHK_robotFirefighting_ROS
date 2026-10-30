#include "session_ui.hpp"
#include "button_style.hpp"
#include <QApplication>
#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QDockWidget>
#include <QEvent>
#include <QInputDialog>
#include <QJsonArray>
#include <QKeyEvent>
#include <QCloseEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMainWindow>
#include <QPushButton>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>

namespace {
class KeyboardDialog : public QDialog
{
public:
  KeyboardDialog(AppClient & client, QWidget * parent) : QDialog(parent), client_(client)
  {
    setWindowTitle(QString::fromUtf8("Bàn phím quét map"));
    auto * layout = new QVBoxLayout(this);
    auto * text = new QLabel(QString::fromUtf8(
      "Bấm vào cửa sổ này để điều khiển\n\n"
      "U   I   O\nJ   K   L\nM   ,   .\n\n"
      "I: tiến · dấu phẩy: lùi · J/L: xoay\nK hoặc Space: dừng\n"
      "Tốc độ 0,10 m/s · xoay 0,8 rad/s\n\n"
      "Đóng khung bàn phím: yêu cầu dừng phiên quét."), this);
    text->setAlignment(Qt::AlignCenter);
    layout->addWidget(text);
    resize(390, 300);
    setModal(false);
  }
protected:
  void keyPressEvent(QKeyEvent * event) override
  {
    const QString key = event->text().toLower();
    if (key.size() == 1 && QString("uiojklm,. ").contains(key)) {
      client_.send("KEY", key); event->accept();
    } else if (event->key() == Qt::Key_Q) {
      client_.stop(); event->accept();
    } else {event->accept();}
  }
  void closeEvent(QCloseEvent * event) override
  {
    client_.stop(); QDialog::closeEvent(event);
  }
  void focusOutEvent(QFocusEvent * event) override
  {
    client_.send("KEY", "k");
    QDialog::focusOutEvent(event);
  }
private:
  AppClient & client_;
};
}

SessionUi::SessionUi(QMainWindow & frame, AppClient & client,
  QComboBox * maps, QPushButton * scan, QPushButton * save, QPushButton * start,
  QPushButton * cancel, QPushButton * stop, QLabel * status, std::function<void(bool)> newSession)
: QObject(&frame), frame_(frame), client_(client), maps_(maps), scan_(scan), save_(save),
  start_(start), cancel_(cancel), stop_(stop), status_(status), newSession_(std::move(newSession))
{
  frame.installEventFilter(this);
  for (auto * button : {scan_, save_, start_, cancel_, stop_}) {installButtonFeedback(button);}
  // Operational config is owned by the package. Disable RViz File/config
  // editing so a loaded config cannot re-enable ungated command tools.
  frame.menuBar()->hide();
  setView(dynamic_cast<rviz_common::VisualizationFrame *>(&frame));
  verify_ = new QPushButton(QString::fromUtf8("Xác minh dừng trên robot"), &frame);
  verify_->setObjectName("verify_stop_button");
  verify_->hide();
  frame.statusBar()->addPermanentWidget(verify_);
  installButtonFeedback(verify_);
  connect(verify_, &QPushButton::clicked, this, [this]() {
    if (!closing_ && !disconnected_ && state_.value("phase").toString() == "BLOCKED") {
      if (client_.send("VERIFY_STOP")) {
        newSessionRequested_ = true;
        verify_->setEnabled(false);
        sentFeedback(QString::fromUtf8("Đã yêu cầu xác minh dừng: chỉ I/O và STOP/OFF/home, không chạy nhiệm vụ"));
      }
    }
  });
  connect(&client_, &AppClient::message, this, [this](const QJsonObject & row) {update(row);});
  connect(&client_, &AppClient::lost, this, [this]() {
    disconnected_ = true; lockCommands();
    stop_->setEnabled(false);
    status_->setText(QString::fromUtf8("Mất kết nối giao diện. Bộ quản lý đang yêu cầu dừng; kiểm tra receipt/log."));
  });
  connect(scan_, &QPushButton::clicked, this, [this]() {
    if (closing_ || disconnected_) {return;}
    if (state_.value("phase").toString() == "BLOCKED") {
      if (client_.send("RECHECK")) {
        std::fprintf(stderr, "UI_RECHECK_REQUEST_SENT\n");
        std::fflush(stderr);
        scan_->setEnabled(false);
        scan_->setText(QString::fromUtf8("Đang yêu cầu kiểm tra…"));
        sentFeedback(QString::fromUtf8("Đã gửi yêu cầu kiểm tra lại · Chờ kết quả"));
      }
    } else if (state_.value("phase").toString() == "IDLE" && client_.send("SCAN")) {
      newSessionRequested_ = true;
      scan_->setText(QString::fromUtf8("Đang yêu cầu quét…"));
      scan_->setEnabled(false); start_->setEnabled(false);
      sentFeedback(QString::fromUtf8("Đã gửi yêu cầu quét map · Chờ khởi động"));
    }
  });
  connect(start_, &QPushButton::clicked, this, [this]() {
    if (!closing_ && !disconnected_ && client_.send("FIRE")) {
      newSessionRequested_ = true;
      start_->setText(QString::fromUtf8("Đang yêu cầu khởi động…"));
      scan_->setEnabled(false); start_->setEnabled(false);
      sentFeedback(QString::fromUtf8("Đã gửi yêu cầu khởi động chữa cháy · Chờ kiểm tra"));
    }
  });
  connect(save_, &QPushButton::clicked, this, [this]() {
    bool ok = false;
    const auto name = QInputDialog::getText(&frame_, QString::fromUtf8("Lưu map"),
      QString::fromUtf8("Tên map (chữ, số, _ và -; không ghi đè):"), QLineEdit::Normal, "", &ok);
    if (ok && client_.send("SAVE", name)) {
      save_->setEnabled(false); save_->setText(QString::fromUtf8("Đang yêu cầu lưu…"));
      sentFeedback(QString::fromUtf8("Đã gửi yêu cầu lưu map · Chờ kết quả"));
    }
  });
  connect(maps_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
    if (index > 0 && client_.send("SELECT", maps_->itemData(index).toJsonObject().value("yaml_path"))) {
      sentFeedback(QString::fromUtf8("Đã gửi lựa chọn map · Chờ xác nhận"));
    }
  });
  const auto cancelAction = [this]() {
    if (cancel_->isEnabled() && !cancelCooling_ && client_.cancel()) {
      cancelCooling_ = true; cancel_->setEnabled(false);
      cancel_->setText(QString::fromUtf8("Đã gửi yêu cầu X"));
      sentFeedback(QString::fromUtf8("Đã gửi yêu cầu hủy tác vụ · Chờ báo cáo operator"));
      QTimer::singleShot(500, this, [this]() {
        cancelCooling_ = false; cancel_->setText(QString::fromUtf8("Dừng tác vụ (X)"));
      });
    }
  };
  connect(cancel_, &QPushButton::clicked, this, cancelAction);
  const auto stopAction = [this]() {
    state_["phase"] = "STOPPING"; lockCommands();
    stop_->setText(QString::fromUtf8("Đang yêu cầu dừng…"));
    sentFeedback(QString::fromUtf8("Đang yêu cầu dừng phiên · Chờ xác nhận STOP và cleanup"));
    client_.stop();
  };
  connect(stop_, &QPushButton::clicked, this, stopAction);
  auto * x = new QShortcut(QKeySequence(Qt::Key_X), &frame_);
  connect(x, &QShortcut::activated, this, cancelAction);
  // Escape belongs to RViz tools, popup menus and dialogs. Only Q stops a
  // session; do not intercept RViz's normal return-to-default-tool shortcut.
  auto * q = new QShortcut(QKeySequence(Qt::Key_Q), &frame_);
  connect(q, &QShortcut::activated, this, stopAction);
  auto * timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() {
    cancel_->setEnabled(ownsSession() && !cancelCooling_ && state_.value("phase").toString() == "RUNNING" && client_.operatorConnected());
  });
  timer->start(100);
}

void SessionUi::sentFeedback(const QString & text)
{
  // This confirms the local send only. The session/receipt area remains the
  // authority for success, failure and physical STOP evidence.
  frame_.statusBar()->showMessage(text, 3000);
}

bool SessionUi::eventFilter(QObject * target, QEvent * event)
{
  if (target == &frame_ && event->type() == QEvent::Close && !closeReady_) {
    closing_ = true; lockCommands(); client_.stop(true);
    status_->setText(QString::fromUtf8("Đang dừng phiên và kiểm tra cleanup. Giữ cửa sổ để xem kết quả."));
    return true;
  }
  return QObject::eventFilter(target, event);
}

void SessionUi::closeAuth()
{
  auto * dialog = auth_.data();
  auth_.clear();
  if (dialog) {dialog->reject();}
}

void SessionUi::update(const QJsonObject & row)
{
  const auto event = row.value("event").toString();
  if (event == "RESTART_UI" && restarting_) {
    closeReady_ = true;
    QTimer::singleShot(0, &frame_, &QWidget::close);
    return;
  }
  if (event == "NEW_SESSION") {
    const auto session = row.value("session").toString();
    if (session.isEmpty() || session == session_) {return;}
    closeAuth();
    session_ = session;
    // Only the acknowledgement of an explicit new start may release STOP.
    // A delayed NEW_SESSION after STOP, or any message after IPC loss, cannot.
    if (newSessionRequested_ && !closing_ && !disconnected_) {stopping_ = false;}
    newSessionRequested_ = false;
    state_ = QJsonObject{{"phase", "STARTING"}, {"stage", "PREFLIGHT"}, {"session", session_}};
    mappingSession_ = row.value("kind").toString() == "mapping";
    newSession_(mappingSession_);
    goalAllowed_ = false;
    tools();
  } else if (event == "SESSION" && row.value("stage").toString() == "RETRY_PI") {
    newSession_(false);
  } else if (event == "AUTH_REQUIRED") {
    if (auth_ || row.value("session").toString() != session_ ||
      stopping_ || closing_ || disconnected_) {return;}
    const auto owner = session_;
    auto * dialog = new QInputDialog(&frame_);
    auth_ = dialog;
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(QString::fromUtf8("Xác thực Pi"));
    dialog->setLabelText(row.value("label").toString());
    dialog->setTextEchoMode(QLineEdit::Password);
    dialog->setWindowModality(Qt::WindowModal);
    connect(dialog, &QDialog::finished, this, [this, dialog, owner](int result) {
      if (auth_.data() != dialog) {return;}
      auth_.clear();
      if (owner != session_ || stopping_ || closing_ || disconnected_) {return;}
      if (result == QDialog::Accepted) {client_.send("AUTH", dialog->textValue());}
      else {client_.stop();}
    });
    // Keep the IPC read handler free to consume STATUS/FINISHED while the
    // user types; an exec/getText nested loop defers that socket's readyRead.
    dialog->open();
  } else if (event == "AUTH_DONE") {
    closeAuth();
  } else if (event == "FINISHED") {
    closeAuth();
    lockCommands();
    if (keyboard_) {keyboard_->hide(); keyboard_->deleteLater(); keyboard_ = nullptr;}
  }
  if (event != "STATUS") {return;}
  const bool changed = row.value("phase") != state_.value("phase") ||
    row.value("stage") != state_.value("stage") || row.value("error") != state_.value("error");
  if (changed) {
    // Record receipt of status without logging command arguments, passwords
    // or pairing material. A click animation alone is not acknowledgement.
    std::fprintf(stderr, "UI_SESSION_STATUS phase=%s stage=%s error_present=%d\n",
      row.value("phase").toString().toUtf8().constData(),
      row.value("stage").toString().toUtf8().constData(), !row.value("error").toString().isEmpty());
    std::fflush(stderr);
  }
  if (row.value("recovery_attempt") != state_.value("recovery_attempt") ||
    row.value("recovery_result") != state_.value("recovery_result")) {
    std::fprintf(stderr, "UI_RECOVERY_STATUS attempt=%d result=%s\n",
      row.value("recovery_attempt").toInt(), row.value("recovery_result").toString().toUtf8().constData());
    std::fflush(stderr);
  }
  state_ = row;
  const auto phase = row.value("phase").toString();
  if (phase != "STARTING" && phase != "MAPPING" && phase != "RUNNING") {closeAuth();}
  if (phase == "STOPPING" || phase == "BLOCKED" || phase == "CHECKING") {
    // BLOCKED permits the read-only retry button. Toggling it off/on on
    // every STATUS cancels a mouse press held across the refresh.
    lockCommands(phase != "BLOCKED");
  }
  const bool idle = phase == "IDLE" && !closing_ && !disconnected_ && !restarting_;
  const bool blocked = phase == "BLOCKED" && !closing_ && !disconnected_ && !restarting_;
  verify_->setVisible(blocked);
  verify_->setEnabled(blocked);
  verify_->setToolTip(QString::fromUtf8("Khởi động I/O để lấy xác nhận dừng mới; có thể gửi OFF/home. Kiểm tra lại chỉ đọc."));
  const bool mapping = phase == "MAPPING" && ownsSession();
  scan_->setText(QString::fromUtf8(phase == "BLOCKED" ? "Kiểm tra lại" : phase == "CHECKING" ? "Đang kiểm tra…" :
    phase == "STARTING" && mappingSession_ ? "Đang mở quét map…" : phase == "MAPPING" ? "Đang quét map" : "Quét map"));
  start_->setText(QString::fromUtf8(phase == "STARTING" && !mappingSession_ ? "Đang khởi động…" :
    phase == "RUNNING" ? "Phiên chữa cháy" : "Khởi động chữa cháy"));
  save_->setText(QString::fromUtf8(row.value("save_pending").toBool() ? "Đang lưu map…" : "Lưu map"));
  stop_->setText(QString::fromUtf8(phase == "STOPPING" ? "Đang dừng phiên…" : "DỪNG PHIÊN (Q)"));
  scan_->setEnabled(idle || blocked); start_->setEnabled(idle && !row.value("selected").toString().isEmpty());
  save_->setEnabled(mapping && !row.value("save_pending").toBool());
  maps_->setEnabled((idle || mapping) && !row.value("save_pending").toBool());
  stop_->setEnabled(!disconnected_ && phase != "IDLE" && phase != "BLOCKED");
  QString text = QString::fromUtf8("Phiên: ") + row.value("stage").toString();
  const auto progress = row.value("progress").toObject();
  if (!progress.isEmpty() && phase == "STARTING") {
    const auto remaining = progress.value("remaining_s");
    if (remaining.isDouble()) {text += QString::fromUtf8(" · còn %1 giây").arg(remaining.toDouble(), 0, 'f', 1);}
    const auto blockers = progress.value("blockers").toArray();
    QStringList names;
    for (const auto & value : blockers) {names << value.toString();}
    if (!names.isEmpty()) {text += QString::fromUtf8("\nĐang chờ dữ liệu phục hồi: ") + names.join(", ");}
  }
  if (!row.value("fault_kind").toString().isEmpty()) {text += " · " + row.value("fault_kind").toString();}
  const auto recoveryResult = row.value("recovery_result").toString();
  if (row.value("recovery_attempt").toInt() > 0 && !recoveryResult.isEmpty()) {
    const auto outcome = QString::fromUtf8(recoveryResult == "CHECKING" ? "ĐANG KIỂM TRA" :
      recoveryResult == "SUCCEEDED" ? "THÀNH CÔNG" : recoveryResult == "CANCELLED" ? "ĐÃ HỦY" : "CHƯA ĐẠT");
    text += QString::fromUtf8("\nKiểm tra lần %1: %2").arg(row.value("recovery_attempt").toInt()).arg(outcome);
  }
  if (poseEnabled()) {text += QString::fromUtf8(" · Đặt vị trí robot bằng 2D Pose Estimate");}
  if (row.value("save_pending").toBool()) {text += QString::fromUtf8(" · Đang lưu map");}
  if (!row.value("error").toString().isEmpty()) {text += "\n" + row.value("error").toString();}
  if (!row.value("recovery_message").toString().isEmpty()) {text += "\n" + row.value("recovery_message").toString();}
  if ((phase == "IDLE" || phase == "BLOCKED") && row.value("session").toString().isEmpty() && row.value("last_stop").isObject()) {
    const auto stop = row.value("last_stop").toObject();
    if (stop.value("session_was_started").isBool() && !stop.value("session_was_started").toBool()) {
      text += QString::fromUtf8("\nPhiên chưa được khởi động.");
    } else {
      text += QString::fromUtf8("\nDừng robot: %1 · Cleanup: %2")
        .arg(stop.value("robot_stop_confirmed").toBool() ? "ĐÃ XÁC NHẬN" : "CHƯA XÁC NHẬN",
        stop.value("cleanup_clean").toBool() ? "SẠCH" : "CẦN KIỂM TRA");
    }
  }
  status_->setText(text);
  {
    const QSignalBlocker block(maps_);
    const auto catalog = row.value("maps").toArray();
    const bool changed = !mapCatalogReady_ || catalog != mapCatalog_;
    // STATUS arrives every 100 ms. Rebuilding an unchanged combo destroys
    // the popup's highlighted row while the operator is trying to select it.
    if (changed) {
      maps_->clear(); maps_->addItem(QString::fromUtf8("Chọn bản đồ…"));
      for (const auto & item : catalog) {
        const auto map = item.toObject();
        maps_->addItem(map.value("name").toString(), map);
        maps_->setItemData(maps_->count() - 1, map.value("yaml_path").toString(), Qt::ToolTipRole);
      }
      mapCatalog_ = catalog; mapCatalogReady_ = true;
    }
    // Preserve keyboard/mouse navigation while an unchanged popup is open.
    // The next status after it closes applies the supervisor's selection,
    // including a rejected selection or a newly saved map.
    if (changed || !maps_->view()->isVisible()) {
      int selected = 0;
      for (int i = 1; i < maps_->count(); ++i) {
        if (maps_->itemData(i).toJsonObject().value("yaml_path") == row.value("selected")) {selected = i; break;}
      }
      if (maps_->currentIndex() != selected) {maps_->setCurrentIndex(selected);}
    }
  }
  if (mapping && !keyboard_) {keyboard_ = new KeyboardDialog(client_, &frame_); keyboard_->show();}
  if (phase == "STOPPING" && keyboard_) {keyboard_->hide();}
  tools();
  if (row.value("close_ready").toBool()) {
    closeReady_ = true; QTimer::singleShot(0, &frame_, &QWidget::close);
  }
}

void SessionUi::updateGoalPermission(bool allowed)
{
  allowed = allowed && ownsSession();
  if (goalAllowed_ != allowed) {goalAllowed_ = allowed; tools();}
}

void SessionUi::lockCommands(bool disableScan)
{
  closeAuth();
  stopping_ = true; newSessionRequested_ = false; goalAllowed_ = false;
  if (disableScan) {scan_->setEnabled(false);}
  save_->setEnabled(false); start_->setEnabled(false);
  cancel_->setEnabled(false); maps_->setEnabled(false);
  if (keyboard_) {keyboard_->hide();}
  tools();
}

bool SessionUi::ownsSession() const
{
  return !closing_ && !stopping_ && !disconnected_ && !session_.isEmpty() &&
    state_.value("session").toString() == session_;
}

bool SessionUi::poseEnabled() const
{
  return ownsSession() && state_.value("phase").toString() == "STARTING" &&
    state_.value("stage").toString() == "POSE";
}

bool SessionUi::goalEnabled() const
{
  return ownsSession() && state_.value("phase").toString() == "RUNNING" && goalAllowed_;
}

void SessionUi::tools()
{
  if (!view_) {return;}
  auto * displays = view_->getManager()->getRootDisplayGroup();
  for (int i = 0; i < displays->numDisplays(); ++i) {
    auto * display = displays->getDisplayAt(i);
    const bool robot = display->getName() == "Robot";
    const bool path = display->getName() == "Nav2 Plan";
    if (!robot && !path) {continue;}
    const bool enabled = viewAllowed() && (robot || !mappingSession_);
    if (display->isEnabled() != enabled) {
      if (!enabled) {display->reset();}
      display->setEnabled(enabled);
    }
  }
  auto * manager = view_->getManager()->getToolManager();
  const bool pose = poseEnabled();
  const bool goal = goalEnabled();
  const QString poseClass = "rviz_default_plugins/SetInitialPose", goalClass = "rviz_default_plugins/SetGoal";
  for (int i = manager->numTools() - 1; i >= 0; --i) {
    const auto id = manager->getTool(i)->getClassId();
    if ((id == poseClass && !pose) || (id == goalClass && !goal) ||
      (id != poseClass && id != goalClass && id != "rviz_default_plugins/MoveCamera" && id != "rviz_default_plugins/Select")) {
      manager->setCurrentTool(manager->getDefaultTool()); manager->removeTool(i);
    }
  }
  bool hasPose = false, hasGoal = false;
  for (int i = 0; i < manager->numTools(); ++i) {
    hasPose |= manager->getTool(i)->getClassId() == poseClass;
    hasGoal |= manager->getTool(i)->getClassId() == goalClass;
  }
  if (pose && !hasPose) {
    auto * tool = manager->addTool(poseClass);
    rviz_common::Config config;
    config.mapSetValue("Class", poseClass);
    config.mapSetValue("Covariance x", 0.25);
    config.mapSetValue("Covariance y", 0.25);
    config.mapSetValue("Covariance yaw", 0.06853891909122467);
    auto topic = config.mapMakeChild("Topic");
    topic.mapSetValue("Value", "/initialpose"); topic.mapSetValue("Depth", 5);
    topic.mapSetValue("History Policy", "Keep Last"); topic.mapSetValue("Reliability Policy", "Reliable");
    topic.mapSetValue("Durability Policy", "Volatile");
    tool->load(config);
  }
  if (goal && !hasGoal) {
    auto * tool = manager->addTool(goalClass);
    rviz_common::Config config;
    config.mapSetValue("Class", goalClass);
    auto topic = config.mapMakeChild("Topic");
    topic.mapSetValue("Value", "/goal_pose"); topic.mapSetValue("Depth", 1);
    topic.mapSetValue("History Policy", "Keep Last"); topic.mapSetValue("Reliability Policy", "Reliable");
    topic.mapSetValue("Durability Policy", "Volatile");
    tool->load(config);
  }
}

void SessionUi::setView(rviz_common::VisualizationFrame * view)
{
  view_ = view;
  if (!view_) {return;}
  // The app owns its camera/status controls. Keep the embedded RViz canvas
  // available for the map instead of duplicating RViz's editor panels.
  view_->menuBar()->hide();
  for (auto * dock : view_->findChildren<QDockWidget *>()) {dock->hide();}
  view_->setHideButtonVisibility(false);
  auto * toolManager = view_->getManager()->getToolManager();
  connect(toolManager, &rviz_common::ToolManager::toolChanged, this, [this, toolManager](rviz_common::Tool * tool) {
    if (!tool) {return;}
    const auto id = tool->getClassId();
    const bool safe = id == "rviz_default_plugins/MoveCamera" || id == "rviz_default_plugins/Select" ||
      (id == "rviz_default_plugins/SetInitialPose" && poseEnabled()) ||
      (id == "rviz_default_plugins/SetGoal" && goalEnabled());
    if (!safe && toolManager->getDefaultTool() && tool != toolManager->getDefaultTool()) {
      toolManager->setCurrentTool(toolManager->getDefaultTool());
    }
  });
  connect(toolManager, &rviz_common::ToolManager::toolAdded, this,
    [this](rviz_common::Tool *) {QTimer::singleShot(0, this, [this]() {tools();});});
  tools();
}

bool SessionUi::viewAllowed() const
{
  const auto stage = state_.value("stage").toString();
  const auto phase = state_.value("phase").toString();
  return ownsSession() && (phase == "MAPPING" || phase == "RUNNING" ||
    (phase == "STARTING" && (stage == "POSE" || stage == "YOLO" || stage == "CONSUMERS" || stage == "WAIT_SLAM")));
}

void SessionUi::prepareRestart()
{
  restarting_ = true;
  lockCommands();
  verify_->setEnabled(false);
  status_->setText(QString::fromUtf8("Phiên đã kết thúc. Đang làm mới cửa sổ RViz; map đã chọn được giữ lại."));
}
