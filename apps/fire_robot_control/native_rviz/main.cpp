#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <QAction>
#include <QApplication>
#include <QBuffer>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDir>
#include <QDockWidget>
#include <QFile>
#include <QFileInfo>
#include <QGroupBox>
#include <QHash>
#include <QHBoxLayout>
#include <QImage>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLabel>
#include <QPainter>
#include <QMainWindow>
#include <QMenuBar>
#include <QProcess>
#include <QScreen>
#include <QSocketNotifier>
#include <QScrollArea>
#include <QPushButton>
#include "button_style.hpp"
#include "goal_admission.hpp"
#include "navigation_recovery_label.hpp"
#include <QStatusBar>
#include <QTimer>
#include <QThread>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>

#include "observer.hpp"
#include "app_client.hpp"
#include "session_ui.hpp"
#include "strict_json.hpp"
#include "wrapping_label.hpp"
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>

using namespace std::chrono_literals;

namespace
{
// The preview is deliberately isolated and publishes only synthetic display
// data. No actions, goals, actuator commands or robot-control topics exist.
class DemoData
{
public:
  explicit DemoData(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    map_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local());
    scan_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS().keep_last(1));
    tf_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    camera_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/yolo/image_annotated/compressed", rclcpp::SensorDataQoS().keep_last(1));
    operator_ = node_->create_publisher<std_msgs::msg::String>(
      "/fire_mission/operator_state", rclcpp::SensorDataQoS().keep_last(1));
    actuator_ = node_->create_publisher<std_msgs::msg::String>(
      "/fire_sensors/actuator_state", rclcpp::SensorDataQoS().keep_last(1));
    QImage frame(640, 360, QImage::Format_RGB32);
    frame.fill(QColor("#3d4248"));
    QPainter painter(&frame);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Sans", 20));
    painter.drawText(frame.rect(), Qt::AlignCenter,
      QString::fromUtf8("CAMERA MÔ PHỎNG\nKhông kết nối robot"));
    painter.end();
    QBuffer buffer(&demoJpeg_);
    buffer.open(QIODevice::WriteOnly);
    frame.save(&buffer, "JPEG", 80);
    demoMap();
  }

  void demoMap()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.1;
    map.info.width = 100;
    map.info.height = 72;
    map.info.origin.position.x = -5.0;
    map.info.origin.position.y = -3.6;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(map.info.width * map.info.height, -1);
    for (unsigned y = 8; y < 64; ++y) {
      for (unsigned x = 8; x < 92; ++x) {
        const bool border = (x == 8 || x == 91 || y == 8 || y == 63);
        const bool wall = ((x == 45 && (y < 25 || y > 44)) ||
          (y == 35 && x > 60 && x < 85));
        map.data[y * map.info.width + x] = (border || wall) ? 100 : 0;
      }
    }
    mapData_ = std::move(map);
  }

  bool selectMap(const QJsonObject & item)
  {
    // Validate content against the catalog snapshot again to avoid silently
    // displaying a changed map. The preview remains in isolated domain 212.
    QFile yaml(item.value("yaml_path").toString());
    QFile image(item.value("image_path").toString());
    if (!yaml.open(QIODevice::ReadOnly) || !image.open(QIODevice::ReadOnly)) {
      return false;
    }
    if (yaml.size() <= 0 || yaml.size() > 65536 ||
      image.size() <= 0 || image.size() > 64 * 1024 * 1024) {
      return false;
    }
    const auto yamlBytes = yaml.readAll();
    const auto imageBytes = image.readAll();
    if (QCryptographicHash::hash(yamlBytes, QCryptographicHash::Sha256).toHex() !=
      item.value("yaml_sha256").toString().toLatin1() ||
      QCryptographicHash::hash(imageBytes, QCryptographicHash::Sha256).toHex() !=
      item.value("image_sha256").toString().toLatin1()) {
      return false;
    }
    QImage picture;
    if (!picture.loadFromData(imageBytes, "PGM") || picture.isNull() ||
      picture.width() != item.value("width").toInt() ||
      picture.height() != item.value("height").toInt() ||
      picture.width() * static_cast<qint64>(picture.height()) > 2000000 ||
      item.value("mode").toString() != "trinary") {
      return false;
    }
    const auto origin = item.value("origin").toArray();
    if (origin.size() != 3) {
      return false;
    }
    const double resolution = item.value("resolution").toDouble();
    const double occupied = item.value("occupied_thresh").toDouble();
    const double free = item.value("free_thresh").toDouble();
    const bool negate = item.value("negate").toInt() == 1;
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = resolution;
    map.info.width = picture.width();
    map.info.height = picture.height();
    map.info.origin.position.x = origin[0].toDouble();
    map.info.origin.position.y = origin[1].toDouble();
    const double yaw = origin[2].toDouble();
    map.info.origin.orientation.z = std::sin(yaw / 2.0);
    map.info.origin.orientation.w = std::cos(yaw / 2.0);
    map.data.resize(map.info.width * map.info.height);
    for (int y = 0; y < picture.height(); ++y) {
      for (int x = 0; x < picture.width(); ++x) {
        const auto pixel = picture.pixelColor(x, picture.height() - y - 1);
        const double gray = (pixel.red() + pixel.green() + pixel.blue()) / (3.0 * 255.0);
        const double probability = negate ? gray : 1.0 - gray;
        map.data[static_cast<size_t>(y) * map.info.width + x] =
          probability > occupied ? 100 : probability < free ? 0 : -1;
      }
    }
    mapData_ = std::move(map);
    return true;
  }

  void publish()
  {
    const auto stamp = node_->now();
    mapData_.header.stamp = stamp;
    map_->publish(mapData_);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.rotation.w = 1.0;
    tf2_msgs::msg::TFMessage transforms;
    transforms.transforms.push_back(t);
    tf_->publish(transforms);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = "base_link";
    scan.angle_min = -3.14F;
    scan.angle_max = 3.14F;
    scan.angle_increment = 6.28F / 179.0F;
    scan.range_min = 0.1F;
    scan.range_max = 8.0F;
    scan.ranges.assign(180, 2.4F);
    scan_->publish(scan);

    sensor_msgs::msg::CompressedImage camera;
    camera.header.stamp = stamp;
    camera.header.frame_id = "camera_frame";
    camera.format = "jpeg";
    camera.data.assign(demoJpeg_.begin(), demoJpeg_.end());
    camera_->publish(camera);
    std_msgs::msg::String state;
    state.data = R"({"schema_version":1,"scope":"R5_SESSION_INTENT_ONLY","task":{"stage":"NAVIGATION","reason":"DỮ LIỆU MÔ PHỎNG"},"yolo_fresh":true,"fire_confirmed":false,"ir_counts":{"left":24000,"center":23800,"right":24200},"ir_source_age_ms":20,"range_confirm_samples":0,"ir_confirm_required":3})";
    operator_->publish(state);
    std_msgs::msg::String actuator;
    actuator.data = R"({"schema_version":1,"mode":"COMMAND_TELEMETRY_ONLY","pump_pwm_commanded_on":false,"servo_phase":"IDLE","receive_age_ms":1})";
    actuator_->publish(actuator);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr actuator_;
  nav_msgs::msg::OccupancyGrid mapData_;
  QByteArray demoJpeg_;
};

QLabel * line(const QString & text, QWidget * parent = nullptr)
{
  return new WrappingLabel(text, parent);
}

class CameraPreview : public QWidget
{
public:
  explicit CameraPreview(bool preview, QWidget * parent = nullptr) : QWidget(parent), preview_(preview)
  {
    setMinimumSize(280, 180);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void setFrame(const QImage & frame)
  {
    frame_ = frame;
    setProperty("frame_fresh", !frame.isNull());
    update();
  }

protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter p(this);
    p.fillRect(rect(), QColor("#3d4248"));
    if (!frame_.isNull()) {
      const QSize scaled = frame_.size().scaled(size(), Qt::KeepAspectRatio);
      const QRect target(QPoint((width() - scaled.width()) / 2,
        (height() - scaled.height()) / 2), scaled);
      p.drawImage(target, frame_);
      return;
    }
    p.setPen(QColor("#ffffff"));
    p.drawText(rect(), Qt::AlignCenter, preview_ ?
      QString::fromUtf8("CAMERA MÔ PHỎNG\nChờ ảnh mẫu") :
      QString::fromUtf8("CAMERA\nChờ ảnh mới"));
    p.setPen(QColor("#bbc0c5"));
    const auto c = rect().center();
    p.drawLine(c.x() - 12, c.y() - 52, c.x() + 12, c.y() - 52);
    p.drawLine(c.x(), c.y() - 64, c.x(), c.y() - 40);
  }

private:
  QImage frame_;
  bool preview_;
};

QPushButton * inactiveButton(const QString & name, QWidget * parent)
{
  auto * button = new QPushButton(name, parent);
  button->setEnabled(false);
  button->setToolTip(QString::fromUtf8("Chức năng vận hành chưa được kết nối trong bản xem thử."));
  return button;
}

SessionUi * configureUi(QMainWindow & frame, DemoData * demo,
  std::unique_ptr<Observer> & observer, const QString & config, bool preview, AppClient * client)
{
  frame.setWindowTitle(preview ? QString::fromUtf8("Fire Robot — BẢN XEM THỬ") :
    QString::fromUtf8(client ? "Fire Robot — TRẠM ĐIỀU KHIỂN" : "Fire Robot — CHỈ QUAN SÁT"));
  QSize initialSize(1280, 720);
  if (auto * screen = frame.screen()) {
    const auto available = screen->availableGeometry().size();
    initialSize = initialSize.boundedTo(QSize(std::max(1, available.width() - 24),
      std::max(1, available.height() - 48)));
  }
  frame.resize(initialSize);
  frame.setStyleSheet(
    "QMainWindow,QWidget { background: #ffffff; color: #252525; }"
    "QToolBar,QDockWidget::title,QGroupBox { background: #f2f3f5; }"
    "QToolBar#sessionBar { background: #b62025; spacing: 5px; padding: 4px; }"
    "QLabel#brand { color: white; background: #b62025; font-weight: bold; padding: 0 10px; }"
    "QLabel#warning { color: #a51c23; font-weight: bold; }"
    "QLabel#section { color: #b62025; font-weight: bold; font-size: 14px; }" + robotButtonStyle());

  auto * bar = new QToolBar(QString::fromUtf8("Điều khiển phiên"), &frame);
  bar->setObjectName("sessionBar");
  bar->setMovable(false);
  auto * brand = new QLabel(preview ? QString::fromUtf8("FIRE ROBOT · DỮ LIỆU MÔ PHỎNG") :
    QString::fromUtf8(client ? "FIRE ROBOT · PTIT" : "FIRE ROBOT · CHỈ QUAN SÁT"), bar);
  brand->setObjectName("brand");
  bar->addWidget(brand);
  auto * scanButton = inactiveButton(QString::fromUtf8("Quét map"), bar);
  auto * saveButton = inactiveButton(QString::fromUtf8("Lưu map"), bar);
  scanButton->setObjectName("scan_button"); saveButton->setObjectName("save_button");
  bar->addWidget(scanButton);
  bar->addWidget(saveButton);
  auto * mapSelect = new QComboBox(bar);
  mapSelect->setObjectName("map_select");
  mapSelect->addItem(preview ? QString::fromUtf8("Map mô phỏng") :
    QString::fromUtf8("Map theo ROS"));
  mapSelect->setEnabled(false);
  bar->addWidget(mapSelect);
  auto * startButton = inactiveButton(QString::fromUtf8("Khởi động chữa cháy"), bar);
  startButton->setObjectName("start_button");
  bar->addWidget(startButton);
  frame.addToolBar(Qt::TopToolBarArea, bar);

  auto * dock = new QDockWidget(QString::fromUtf8("Camera / YOLO"), &frame);
  dock->setObjectName("camera_dock");
  dock->setFeatures(QDockWidget::NoDockWidgetFeatures);
  auto * content = new QWidget(dock);
  auto * layout = new QVBoxLayout(content);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);
  auto * cameraWidget = new CameraPreview(preview, content);
  cameraWidget->setObjectName("camera_widget");
  layout->addWidget(cameraWidget, 3);
  auto * statusScroll = new QScrollArea(content);
  statusScroll->setObjectName("status_scroll");
  statusScroll->setWidgetResizable(true);
  statusScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  statusScroll->setFrameShape(QFrame::NoFrame);
  statusScroll->setMinimumSize(0, 90);
  statusScroll->setMaximumHeight(330);
  auto * statusContent = new QWidget(statusScroll);
  auto * statusLayout = new QVBoxLayout(statusContent);
  statusLayout->setContentsMargins(0, 0, 0, 0);
  statusLayout->setSpacing(6);
  auto * status = line(QString::fromUtf8("TRẠNG THÁI ROBOT"), content);
  status->setObjectName("section");
  statusLayout->addWidget(status);
  auto * taskLabel = line(QString::fromUtf8("Chờ báo cáo operator"), content);
  taskLabel->setObjectName("operator_status");
  auto * fireLabel = line(QString::fromUtf8("YOLO: chưa có dữ liệu"), content);
  auto * irLabel = line(QString::fromUtf8("IR: chưa có dữ liệu"), content);
  auto * actuatorLabel = line(QString::fromUtf8("Bơm / Servo: chưa có dữ liệu"), content);
  actuatorLabel->setObjectName("actuator_status");
  statusLayout->addWidget(taskLabel);
  statusLayout->addWidget(fireLabel);
  statusLayout->addWidget(irLabel);
  statusLayout->addWidget(actuatorLabel);
  auto * warning = line(QString::fromUtf8(client ? "Đang kết nối bộ quản lý phiên…" :
    "BẢN XEM THỬ: nút vận hành chưa hoạt động"), content);
  warning->setObjectName("warning");
  statusLayout->insertWidget(1, warning);
  auto * mapStatus = line(preview ? QString::fromUtf8("Map, robot và Lidar mô phỏng") :
    QString::fromUtf8("Chỉ quan sát · chưa có chức năng điều khiển"), content);
  statusLayout->addWidget(mapStatus);
  statusLayout->addStretch();
  statusScroll->setWidget(statusContent);
  layout->addWidget(statusScroll, 2);
  auto * buttons = new QHBoxLayout();
  auto * cancelButton = inactiveButton(QString::fromUtf8("Dừng tác vụ (X)"), content);
  auto * stopButton = inactiveButton(QString::fromUtf8("DỪNG PHIÊN (Q)"), content);
  cancelButton->setObjectName("cancel_button"); stopButton->setObjectName("stop_button");
  buttons->addWidget(cancelButton);
  buttons->addWidget(stopButton);
  layout->addLayout(buttons);
  dock->setWidget(content);
  frame.addDockWidget(Qt::RightDockWidgetArea, dock);
  frame.resizeDocks({dock}, {580}, Qt::Horizontal);
  SessionUi * sessionUi = nullptr;
  if (client) {
    for (auto * button : {scanButton, saveButton, startButton, cancelButton, stopButton}) {
      button->setToolTip("");
    }
    mapStatus->setText(QString::fromUtf8("Chọn map → Khởi động → đặt Initial Pose → chờ AUTO armed\nXe: nhìn từ trên xuống · Đường xanh: kế hoạch Nav2 gần nhất"));
    sessionUi = new SessionUi(frame, *client, mapSelect, scanButton, saveButton,
      startButton, cancelButton, stopButton, warning, [&observer](bool mapping) {
        observer.reset();
        (void)mapping; // The view lifecycle attaches ROS only after seal.
      });
  }
  QObject::connect(mapSelect, QOverload<int>::of(&QComboBox::currentIndexChanged),
    &frame, [demo, mapSelect, mapStatus](int index) {
      if (!demo) {
        return;
      }
      if (index == 0) {
        demo->demoMap();
        mapStatus->setText(QString::fromUtf8("Map, robot và Lidar mô phỏng"));
        return;
      }
      const auto item = mapSelect->itemData(index).toJsonObject();
      if (demo->selectMap(item)) {
        mapStatus->setText(QString::fromUtf8("Đang xem map đã lưu · robot/Lidar mô phỏng"));
        std::fprintf(stderr, "PREVIEW_MAP=LOADED\n");
      } else {
        mapStatus->setText(QString::fromUtf8("Không đọc được map hoặc map đã thay đổi"));
        std::fprintf(stderr, "PREVIEW_MAP=INVALID\n");
        demo->demoMap();
      }
    });
  // Catalog is read-only and runs outside the UI event loop. It does not
  // activate the robot or change the stored map selection.
  const QDir appDir(QFileInfo(config).absoluteDir().filePath(".."));
  const QDir repoDir(appDir.filePath("../.."));
  auto * catalog = new QProcess(&frame);
  catalog->setProgram("/usr/bin/python3");
  catalog->setArguments({
    appDir.filePath("services/map_catalog.py"),
    repoDir.filePath("ros2_ws/src/fire_robot_navigation/maps"),
    repoDir.filePath(".codex_qa_backups/r5_operational_candidate_20261026/ros/ros2_ws/src/fire_robot_navigation/maps")});
  QObject::connect(catalog, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
    &frame, [catalog, mapSelect, mapStatus, preview](int code, QProcess::ExitStatus exitStatus) {
      if (code != 0 || exitStatus != QProcess::NormalExit) {
        mapStatus->setText(QString::fromUtf8("Không đọc được danh sách bản đồ"));
        return;
      }
      const auto data = QJsonDocument::fromJson(catalog->readAllStandardOutput());
      if (!data.isObject() || !data.object().value("maps").isArray()) {
        mapStatus->setText(QString::fromUtf8("Danh sách bản đồ không hợp lệ"));
        return;
      }
      for (const auto & value : data.object().value("maps").toArray()) {
        const auto item = value.toObject();
        const QString path = item.value("yaml_path").toString();
        const QString parent = path.contains(".codex_qa_backups") ? "R5" : "Workspace";
        mapSelect->addItem(item.value("name").toString() + " (" + parent + ")", item);
        mapSelect->setItemData(mapSelect->count() - 1, path, Qt::ToolTipRole);
      }
      mapSelect->setEnabled(preview);
      if (const char * selected = preview ? std::getenv("FIRE_ROBOT_PREVIEW_SELECT_MAP") : nullptr) {
        for (int i = 1; i < mapSelect->count(); ++i) {
          if (mapSelect->itemData(i).toJsonObject().value("name").toString() ==
            QString::fromLocal8Bit(selected)) {
            mapSelect->setCurrentIndex(i);
            break;
          }
        }
      }
    });
  if (!client) {catalog->start();}
  auto * refresh = new QTimer(&frame);
  QObject::connect(refresh, &QTimer::timeout, &frame,
    [&observer, cameraWidget, taskLabel, fireLabel, irLabel, actuatorLabel, sessionUi]() {
      const auto snap = observer ? observer->snapshot() : Observer::Snapshot{};
      const auto now = Observer::Clock::now();
      const auto age = [now](Observer::Clock::time_point received) {
          return received == Observer::Clock::time_point{} || received > now ? 1e9 :
            std::chrono::duration<double>(now - received).count();
        };
      const double imageAge = age(snap.imageReceived);
      cameraWidget->setFrame(snap.imageValid && imageAge <= 0.35 &&
        snap.imageSourceAgeMs + imageAge * 1000 <= 300 ? snap.image : QImage());
      const auto doc = strictJsonObject(snap.operatorJson.toUtf8());
      const auto report = doc.value_or(QJsonObject());
      if (sessionUi) {
        sessionUi->updateGoalPermission(appGoalAllowed(report, age(snap.operatorReceived)));
      }
      if (age(snap.operatorReceived) > 0.5 || !doc ||
        report.value("schema_version").toInt() != 1 ||
        report.value("scope").toString() != "R5_SESSION_INTENT_ONLY") {
        taskLabel->setText(QString::fromUtf8("Operator: chờ báo cáo mới"));
        fireLabel->setText(QString::fromUtf8("YOLO: chưa có dữ liệu mới"));
        irLabel->setText(QString::fromUtf8("IR: chưa có dữ liệu mới"));
      } else {
        const auto task = report.value("task").toObject();
        const QString stage = task.value("stage").toString();
        const QString reason = task.value("reason").toString();
        const auto shutdown = report.value("shutdown").toObject();
        if (shutdown.value("requested").toBool()) {
          taskLabel->setText(shutdown.value("done").toBool() ?
            (shutdown.value("confirmed").toBool() ? QString::fromUtf8("Đã xác nhận dừng") :
            QString::fromUtf8("Chưa xác nhận dừng")) : QString::fromUtf8("Đang dừng phiên"));
        } else if (!report.value("fault").isNull() && !report.value("fault").isUndefined()) {
          taskLabel->setText(QString::fromUtf8("Có lỗi — kiểm tra chi tiết"));
        } else if (report.value("input_hold").toBool() ||
          report.value("runtime_rebind").toObject().value("held").toBool()) {
          taskLabel->setText(QString::fromUtf8("Tạm dừng — chờ phục hồi"));
        } else if (report.value("app_goal_admission").toObject().value("cancel_pending").toBool()) {
          const auto cancelReason = report.value("reason").toString() + " " +
            report.value("blocked_goal_cancel").toObject().value("state").toString();
          taskLabel->setText(cancelReason.contains("CANCEL_SEND_FAILED") ||
            cancelReason.contains("CANCEL_REJECTED") || cancelReason.contains("CANCEL_UNCONFIRMED") ?
            QString::fromUtf8("Chưa xác nhận hủy Goal · bấm Q để dừng và kiểm tra phiên") :
            QString::fromUtf8("Đang hủy tác vụ · chờ dừng / OFF / HOME"));
        } else if (!navigationRecoveryLabel(report.value("app_navigation_recovery").toObject()).isEmpty()) {
          taskLabel->setText(navigationRecoveryLabel(report.value("app_navigation_recovery").toObject()));
        } else {
          static const QHash<QString, QString> labels = {
            {"STARTUP", QString::fromUtf8("Khởi động")},
            {"NAVIGATION", QString::fromUtf8("Điều hướng")},
            {"ALIGN", QString::fromUtf8("Căn tâm")},
            {"IR", QString::fromUtf8("Xử lý IR")},
            {"SPRAY", QString::fromUtf8("Xử lý phun")},
            {"RETURN_NAV2", QString::fromUtf8("Trả quyền Nav2")},
            {"STOP", QString::fromUtf8("Dừng")}};
          taskLabel->setText(labels.value(stage, QString::fromUtf8("Đang chờ")) +
            (reason.isEmpty() ? "" : " · " + reason));
        }
        const auto fire = report.value("fire_confirmed");
        if (report.value("yolo_fresh").toBool() && fire.isBool()) {
          fireLabel->setText(fire.toBool() ? QString::fromUtf8("YOLO: đã xác nhận lửa") :
            QString::fromUtf8("YOLO: chưa xác nhận lửa"));
        } else {
          fireLabel->setText(QString::fromUtf8("YOLO: chưa có dữ liệu mới"));
        }
        const auto counts = report.value("ir_counts").toObject();
        const auto irAge = report.value("ir_source_age_ms");
        if (counts.value("left").isDouble() && counts.value("center").isDouble() &&
          counts.value("right").isDouble() && irAge.isDouble() && irAge.toDouble() >= 0 &&
          irAge.toDouble() + age(snap.operatorReceived) * 1000 <= 500 &&
          report.value("range_confirm_samples").isDouble() &&
          report.value("ir_confirm_required").isDouble()) {
          irLabel->setText(QString::fromUtf8("IR lọc: %1 / %2 / %3 · xác nhận %4/%5")
            .arg(counts.value("left").toInt()).arg(counts.value("center").toInt())
            .arg(counts.value("right").toInt()).arg(report.value("range_confirm_samples").toInt())
            .arg(report.value("ir_confirm_required").toInt()));
        } else {
          irLabel->setText(QString::fromUtf8("IR: chưa có dữ liệu hợp lệ"));
        }
      }
      const auto actDoc = strictJsonObject(snap.actuatorJson.toUtf8());
      const auto act = actDoc.value_or(QJsonObject());
      const auto pwm = act.value("pump_pwm_commanded_on");
      const auto actSourceAge = act.value("receive_age_ms");
      const double totalAge = actSourceAge.toDouble(1e9) + age(snap.actuatorReceived) * 1000;
      if (!actDoc || act.value("schema_version").toInt() != 1 ||
        act.value("mode").toString() != "COMMAND_TELEMETRY_ONLY" ||
        !pwm.isBool() || !actSourceAge.isDouble() || actSourceAge.toDouble() < 0 ||
        totalAge > 500) {
        actuatorLabel->setText(QString::fromUtf8("Bơm / Servo: chưa có dữ liệu mới"));
      } else {
        actuatorLabel->setText(QString::fromUtf8("ESP32 PWM: %1 · Servo: %2")
          .arg(pwm.toBool() ? "ON" : "OFF", act.value("servo_phase").toString("—")));
      }
    });
  refresh->start(100);
  frame.statusBar()->showMessage(preview ? QString::fromUtf8(
    "DỮ LIỆU MÔ PHỎNG · Không kết nối Pi · Không điều khiển robot") : QString::fromUtf8(
    client ? "Bộ quản lý phiên độc lập · X hủy tác vụ · Q dừng phiên" :
    "CHỈ QUAN SÁT · Không có nút điều khiển hoạt động"));
  return sessionUi;
}
}  // namespace

// A mapping view never owns the frontend lease or motion controls. Keeping
// RViz in a separate process means slow OpenGL/plugin startup cannot freeze
// the main window's heartbeat and STOP button.
class MappingViewKeys : public QObject
{
protected:
  bool eventFilter(QObject *, QEvent * event) override
  {
    if (event->type() != QEvent::KeyPress) {return false;}
    auto * key = static_cast<QKeyEvent *>(event);
    if (key->isAutoRepeat()) {return false;}
    if (key->key() == Qt::Key_Q) {
      std::fputs("APP_CHILD_STOP\n", stdout); std::fflush(stdout); return true;
    }
    return false;
  }
};

static int mappingViewMain(int argc, char ** argv)
{
  if (argc != 4) {return 2;}
  bool ok = false;
  const auto parent = QString::fromLocal8Bit(argv[3]).toLongLong(&ok);
  if (!ok || parent <= 1 || ::prctl(PR_SET_PDEATHSIG, SIGKILL) || ::getppid() != parent) {return 2;}
  int qt_argc = 1, ros_argc = 1;
  QApplication app(qt_argc, argv);
  if (QGuiApplication::platformName() != "xcb") {return 2;}
  MappingViewKeys keys;
  app.installEventFilter(&keys);
#ifdef FIRE_ROBOT_GUI_QA
  if (qgetenv("FIRE_ROBOT_GUI_QA") == "child_crash") {QThread::msleep(500); return 3;}
  if (qgetenv("FIRE_ROBOT_GUI_QA") == "slow_workflow" ||
    qgetenv("FIRE_ROBOT_GUI_QA") == "slow_stop") {QThread::msleep(3300);}
#endif
  try {
    rclcpp::init(ros_argc, argv);
    auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_control_preview");
    rviz_common::VisualizationFrame view(abstraction);
    view.setApp(&app); view.setSplashPath("");
    view.initialize(abstraction, QString::fromLocal8Bit(argv[2]));
    view.menuBar()->hide();
    for (auto * dock : view.findChildren<QDockWidget *>()) {dock->hide();}
    view.setHideButtonVisibility(false);
    auto * manager = view.getManager()->getToolManager();
    for (int i = manager->numTools() - 1; i >= 0; --i) {
      const auto id = manager->getTool(i)->getClassId();
      if (id != "rviz_default_plugins/MoveCamera" && id != "rviz_default_plugins/Select") {
        manager->setCurrentTool(manager->getDefaultTool()); manager->removeTool(i);
      }
    }
    QObject::connect(manager, &rviz_common::ToolManager::toolChanged, &view,
      [manager](rviz_common::Tool * tool) {
        if (!tool) {return;}
        const auto id = tool->getClassId();
        if (id != "rviz_default_plugins/MoveCamera" && id != "rviz_default_plugins/Select" &&
          manager->getDefaultTool() && tool != manager->getDefaultTool()) {
          manager->setCurrentTool(manager->getDefaultTool());
        }
      });
    QObject::connect(manager, &rviz_common::ToolManager::toolAdded, &view,
      [manager, &view](rviz_common::Tool *) {
        QTimer::singleShot(0, &view, [manager]() {
          for (int i = manager->numTools() - 1; i >= 0; --i) {
            const auto id = manager->getTool(i)->getClassId();
            if (id != "rviz_default_plugins/MoveCamera" && id != "rviz_default_plugins/Select") {
              manager->setCurrentTool(manager->getDefaultTool()); manager->removeTool(i);
            }
          }
        });
      });
    auto * displays = view.getManager()->getRootDisplayGroup();
    for (int i = 0; i < displays->numDisplays(); ++i) {
      auto * display = displays->getDisplayAt(i);
      if (display->getName() == "Robot") {display->setEnabled(true);}
      if (display->getName() == "Nav2 Plan") {display->setEnabled(false);}
    }
    QSocketNotifier parentPipe(STDIN_FILENO, QSocketNotifier::Read);
    QObject::connect(&parentPipe, &QSocketNotifier::activated, &app, [&app, &view](int) {
      char input[64];
      const auto count = ::read(STDIN_FILENO, input, sizeof(input));
      const QByteArray commands(input, count > 0 ? count : 0);
      if (count <= 0 || commands.contains("STOP")) {app.quit();}
      else if (commands.contains("SHOW")) {
        view.show();
#ifdef FIRE_ROBOT_GUI_QA
        if (qgetenv("FIRE_ROBOT_GUI_QA") == "child_key") {
          QTimer::singleShot(500, &view, [&view]() {
            QKeyEvent key(QEvent::KeyPress, Qt::Key_Q, Qt::NoModifier);
            QCoreApplication::sendEvent(&view, &key);
          });
        }
#endif
      }
    });
    std::fprintf(stdout, "APP_CHILD_READY %llu\n", static_cast<unsigned long long>(view.winId()));
    std::fflush(stdout);
    const int result = app.exec();
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return result;
  } catch (const std::exception &) {
    std::fprintf(stderr, "APP_CHILD_INIT_FAILED\n");
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 2;
  }
}

int main(int argc, char ** argv)
{
  if (argc >= 2 && std::string(argv[1]) == "--mapping-view") {return mappingViewMain(argc, argv);}
  if ((argc != 3 && argc != 4) || (std::string(argv[1]) != "--preview" && std::string(argv[1]) != "--observe" && std::string(argv[1]) != "--control")) {
    std::fprintf(stderr, "Usage: fire_robot_control_preview --preview|--observe config | --control config bootstrap-fd\n");
    return 2;
  }
  const bool preview = std::string(argv[1]) == "--preview";
  const bool control = std::string(argv[1]) == "--control";
  std::optional<FrontendBootstrap> bootstrap;
  if (control) {
    if (argc != 4) {return 2;}
    bool valid = false;
    const int fd = QString::fromLocal8Bit(argv[3]).toInt(&valid);
    if (!valid) {return 2;}
    bootstrap = readFrontendBootstrap(fd);
    ::close(fd);
    if (!bootstrap) {return 2;}
  } else if (argc != 3) {return 2;}
  const QString config = QString::fromLocal8Bit(argv[2]);
  // Set isolation before initializing DDS; preview cannot attach to the robot.
  if (preview && (setenv("ROS_DOMAIN_ID", "212", 1) || setenv("ROS_LOCALHOST_ONLY", "1", 1))) {
    std::fprintf(stderr, "Failed to isolate the preview ROS domain.\n");
    return 2;
  }
  int ros_argc = 1;
  int qt_argc = 1;
  QApplication app(qt_argc, argv);
  std::unique_ptr<AppClient> client;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> abstraction;
  std::unique_ptr<DemoData> demo;
  std::unique_ptr<Observer> observer;
  std::unique_ptr<QMainWindow> outer;
  rviz_common::VisualizationFrame * rviz = nullptr;
  if (control) {
    outer = std::make_unique<QMainWindow>();
    auto * placeholder = new QLabel(QString::fromUtf8("Bản đồ sẽ hiện sau khi Pi sẵn sàng"), outer.get());
    placeholder->setAlignment(Qt::AlignCenter);
    outer->setCentralWidget(placeholder);
  } else {
    rclcpp::init(ros_argc, argv);
    abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_control_preview");
    auto view = std::make_unique<rviz_common::VisualizationFrame>(abstraction);
    rviz = view.get();
    rviz->setApp(&app); rviz->setSplashPath(""); rviz->initialize(abstraction, config);
    outer = std::move(view);
    if (preview) {demo = std::make_unique<DemoData>(abstraction->get_raw_node());}
    observer = std::make_unique<Observer>(preview);
  }
  int rc = 0;
  {
    QMainWindow & frame = *outer;
    if (bootstrap) {client = std::make_unique<AppClient>(*bootstrap);}
    auto * sessionUi = configureUi(frame, demo.get(), observer, config, preview, client.get());
    QPointer<QProcess> mappingChild;
    QWidget * mappingContainer = nullptr;
    bool childStopping = false;
    QByteArray childOutput;
    auto detachView = [&]() {
      if (!control) {return;}
      const bool hadView = rviz || observer || abstraction || mappingContainer || (mappingChild && !childStopping);
      if (!hadView && !rclcpp::ok()) {return;}
      if (mappingChild && !childStopping) {
        childStopping = true;
        QPointer<QProcess> child = mappingChild;
        child->write("STOP\n");
        QTimer::singleShot(1200, &frame, [child]() {if (child && child->state() != QProcess::NotRunning) {child->kill();}});
        std::fprintf(stderr, "APP_VIEW_CHILD_STOP_REQUESTED\n");
      }
      if (mappingContainer) {
        auto * widget = frame.takeCentralWidget();
        mappingContainer = nullptr;
        if (widget) {widget->deleteLater();}
        if (auto * dock = frame.findChild<QDockWidget *>("camera_dock")) {dock->setMaximumWidth(QWIDGETSIZE_MAX);}
      }
      observer.reset();
      sessionUi->setView(nullptr);
      if (rviz) {delete frame.takeCentralWidget(); rviz = nullptr;}
      abstraction.reset();
      if (rclcpp::ok()) {rclcpp::shutdown();}
      std::fprintf(stderr, "APP_VIEW_DETACHED\n");
    };
    bool attaching = false;
    bool attachRequested = false;
    bool viewEverStarted = false;
    if (control) {
      QObject::connect(client.get(), &AppClient::message, &frame, [&](const QJsonObject & row) {
        const auto event = row.value("event").toString();
        if (event == "NEW_SESSION" && !attaching) {attachRequested = false;}
        if (event != "STATUS" && event != "VIEW_INIT_GRANTED") {return;}
        if (!sessionUi->viewAllowed()) {
          const auto phase = row.value("phase").toString();
          if (mappingChild && !childStopping && (phase == "STOPPING" || phase == "BLOCKED" || phase == "IDLE")) {detachView();}
          if (viewEverStarted && !attaching && !mappingChild && (phase == "IDLE" || phase == "BLOCKED") && !sessionUi->restarting() && !sessionUi->closing()) {
            if (client->send("RECYCLE_UI")) {sessionUi->prepareRestart();}
          }
          return;
        }
        if (rviz || mappingChild || attaching) {return;}
        if (event != "VIEW_INIT_GRANTED") {
          if (!attachRequested && client->send(sessionUi->mappingView() ? "VIEW_ASYNC_INIT" : "VIEW_INIT", sessionUi->sessionId())) {attachRequested = true;}
          return;
        }
        if (row.value("session").toString() != sessionUi->sessionId()) {return;}
        if (sessionUi->mappingView()) {
          viewEverStarted = true;
          childStopping = false;
          childOutput.clear();
          auto * child = new QProcess(&frame);
          mappingChild = child;
          const auto owner = sessionUi->sessionId();
          QObject::connect(child, &QProcess::readyReadStandardOutput, &frame, [&, child, owner]() {
            childOutput += child->readAllStandardOutput();
            if (childOutput.size() > 16384) {client->stop(); childStopping = true; child->kill(); return;}
            while (childOutput.contains('\n')) {
              const auto newline = childOutput.indexOf('\n');
              const auto line = childOutput.left(newline).trimmed();
              childOutput.remove(0, newline + 1);
              if (line == "APP_CHILD_STOP") {std::fprintf(stderr, "APP_CHILD_STOP_RECEIVED\n"); client->stop(); continue;}
              if (!line.startsWith("APP_CHILD_READY ")) {continue;}
              bool ok = false;
              const auto id = line.mid(16).toULongLong(&ok);
              if (!ok || !id || childStopping || mappingContainer || !sessionUi->viewAllowed() ||
                sessionUi->sessionId() != owner) {client->stop(); childStopping = true; child->kill(); return;}
              try {
                auto * window = QWindow::fromWinId(static_cast<WId>(id));
                if (!window) {throw std::runtime_error("Cannot embed RViz window");}
                mappingContainer = QWidget::createWindowContainer(window, &frame);
                mappingContainer->setMinimumSize(320, 180);
                mappingContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                frame.setCentralWidget(mappingContainer);
                if (auto * dock = frame.findChild<QDockWidget *>("camera_dock")) {
                  dock->setMaximumWidth(std::max(280, frame.width() * 55 / 100));
                  frame.resizeDocks({dock}, {frame.width() / 2}, Qt::Horizontal);
                }
                child->write("SHOW\n");
                rclcpp::init(ros_argc, argv);
                observer = std::make_unique<Observer>(false, true);
                if (!client->send("VIEW_READY", owner)) {throw std::runtime_error("Cannot admit view");}
                std::fprintf(stderr, "APP_VIEW_ATTACHED\n");
              } catch (const std::exception &) {
                std::fprintf(stderr, "APP_VIEW_ATTACH_FAILED\n");
                client->stop(); detachView();
              }
            }
          });
          QObject::connect(child, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), &frame,
            [&, child](int, QProcess::ExitStatus) {
              const bool unexpected = !childStopping;
              if (mappingChild == child) {mappingChild = nullptr;}
              child->deleteLater();
              if (unexpected) {std::fprintf(stderr, "APP_VIEW_CHILD_EXITED\n"); client->stop(); detachView();}
            });
          QObject::connect(child, &QProcess::errorOccurred, &frame, [&, child](QProcess::ProcessError error) {
            if (error == QProcess::FailedToStart && mappingChild == child) {
              std::fprintf(stderr, "APP_VIEW_CHILD_START_FAILED\n");
              mappingChild = nullptr; child->deleteLater(); client->stop(); detachView();
            }
          });
          child->setProcessChannelMode(QProcess::ForwardedErrorChannel);
          child->start(QCoreApplication::applicationFilePath(),
            {"--mapping-view", config, QString::number(::getpid())});
          return;
        }
        attaching = true;
        viewEverStarted = true;
#ifdef FIRE_ROBOT_GUI_QA
        if (qgetenv("FIRE_ROBOT_GUI_QA") == "slow_workflow") {QThread::msleep(900);}
#endif
        try {
          rclcpp::init(ros_argc, argv);
          abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_control_preview");
          auto pending = std::make_unique<rviz_common::VisualizationFrame>(abstraction);
          pending->setApp(&app); pending->setSplashPath(""); pending->initialize(abstraction, config);
          rviz = pending.release();
          rviz->menuBar()->hide();
          frame.setCentralWidget(rviz);
          sessionUi->setView(rviz);
          rviz->show();
          if (sessionUi->viewAllowed()) {
            observer = std::make_unique<Observer>(false, sessionUi->mappingView());
            client->send("VIEW_READY", sessionUi->sessionId());
          }
          std::fprintf(stderr, "APP_VIEW_ATTACHED\n");
        } catch (const std::exception &) {
          client->stop(); detachView();
          std::fprintf(stderr, "APP_VIEW_ATTACH_FAILED\n");
        }
        attaching = false;
      });
      QObject::connect(client.get(), &AppClient::lost, &frame, [&]() {if (!attaching) {detachView();}});
    }
    QTimer timer;
    // RViz already attaches its node to an executor; adding it to a second
    // executor raises and terminates the application.
    QObject::connect(&timer, &QTimer::timeout, [&]() {
      if (demo) {demo->publish();}
      if (mappingContainer) {
        if (auto * dock = frame.findChild<QDockWidget *>("camera_dock")) {
          dock->setMaximumWidth(std::max(280, frame.width() * 55 / 100));
        }
      }
    });
    timer.start(200);
    frame.show();
#ifdef FIRE_ROBOT_GUI_QA
    // This code is compiled only into the isolated QA fixture. Preflight
    // rejects ROS domain 211 before SSH; the normal app has no auto-clicks.
    if (control && qEnvironmentVariableIsSet("FIRE_ROBOT_GUI_QA")) {
      auto * qa = new QTimer(&frame);
      auto * qaStep = new int(0);
      auto * qaLoops = new int(0);
      auto * idleAt = new std::chrono::steady_clock::time_point{};
      const QByteArray mode = qgetenv("FIRE_ROBOT_GUI_QA");
      const QString qaState = qEnvironmentVariable("FIRE_ROBOT_GUI_QA_STATE");
      if (!qaState.isEmpty()) {
        QFile saved(qaState);
        if (saved.open(QIODevice::ReadOnly)) {
          const auto values = saved.readAll().split(' ');
          if (values.size() == 2) {*qaStep = values[0].toInt(); *qaLoops = values[1].toInt();}
        }
      }
      QObject::connect(qa, &QTimer::timeout, &frame, [&frame, &rviz, qa, qaStep, qaLoops, idleAt, mode]() {
        const QString qaMap = mode == "cycles" ? QString("qa_cycle_%1").arg(*qaLoops) : "qa_new_map";
        auto * scan = frame.findChild<QPushButton *>("scan_button");
        auto * start = frame.findChild<QPushButton *>("start_button");
        auto * save = frame.findChild<QPushButton *>("save_button");
        auto * stop = frame.findChild<QPushButton *>("stop_button");
        auto * maps = frame.findChild<QComboBox *>("map_select");
        auto * status = frame.findChild<QLabel *>("warning");
        if (!scan || !start || !save || !stop || !maps || !status) {return;}
        if (*qaStep == 0 && scan->isEnabled()) {
          if (mode == "idle") {
            std::fprintf(stderr, "GUI_IDLE_ROS_CONTEXT=%d\n", rclcpp::ok() ? 1 : 0);
            if (const char * path = std::getenv("FIRE_ROBOT_GUI_QA_CAPTURE")) {frame.grab().save(QString::fromLocal8Bit(path));}
            *qaStep = 90;
            QTimer::singleShot(2500, &frame, [&frame]() {frame.close();});
          } else if (mode == "scan") {scan->click(); *qaStep = 2; std::fprintf(stderr, "GUI_FIXTURE_SCAN\n");}
          else if (mode == "fire" && maps->count() > 1) {maps->setCurrentIndex(1); *qaStep = 1;}
          else if (mode == "workflow" || mode == "slow_workflow" || mode == "cycles") {scan->click(); *qaStep = 10; std::fprintf(stderr, "GUI_WORKFLOW_SCAN\n");}
          else if (mode == "slow_stop") {scan->click(); *qaStep = 20;}
          else if (mode == "child_crash" || mode == "child_key") {scan->click(); *qaStep = 21;}
        } else if (*qaStep == 1 && start->isEnabled()) {
          start->click(); *qaStep = 2; std::fprintf(stderr, "GUI_FIXTURE_FIRE\n");
        } else if (*qaStep == 2 && scan->isEnabled() && status->text().contains("UNREVIEWED_ROS_DOMAIN_OR_LOCALHOST")) {
          std::fprintf(stderr, "GUI_FIXTURE_PREFLIGHT_REJECTED\n");
          *qaStep = 3; qa->stop(); frame.close();
        } else if (*qaStep == 20 && stop->isEnabled() && status->text().contains("WAIT_SLAM")) {
          *qaStep = 21;
          QTimer::singleShot(500, stop, [stop]() {stop->click(); std::fprintf(stderr, "GUI_SLOW_STOP\n");});
        } else if (*qaStep == 21 && scan->isEnabled()) {
          *qaStep = 22; qa->stop(); frame.close();
        } else if (*qaStep == 10 && save->isEnabled() && qEnvironmentVariableIsSet("FIRE_ROBOT_GUI_QA_MAPPING_CAPTURE")) {
          *qaStep = 101;
          QTimer::singleShot(1200, &frame, [qaStep]() {*qaStep = 102;});
        } else if ((*qaStep == 10 || *qaStep == 102) && save->isEnabled()) {
#ifdef FIRE_ROBOT_GUI_QA
          for (auto * widget : QApplication::topLevelWidgets()) {
            if (widget->windowTitle() != QString::fromUtf8("Bàn phím quét map")) {continue;}
            for (auto * label : widget->findChildren<QLabel *>()) {
              if (label->text().contains(QString::fromUtf8("Tốc độ 0,10 m/s · xoay 0,8 rad/s"))) {
                std::fprintf(stderr, "GUI_MAPPING_SPEED_LABEL_OK\n");
              }
            }
          }
#endif
          auto * canvas = frame.centralWidget();
          auto * camera = frame.findChild<QWidget *>("camera_widget");
          std::fprintf(stderr, "GUI_MAPPING_VIEW_SIZE=%dx%d CAMERA_SIZE=%dx%d\n",
            canvas ? canvas->width() : 0, canvas ? canvas->height() : 0,
            camera ? camera->width() : 0, camera ? camera->height() : 0);
          if (const char * path = std::getenv("FIRE_ROBOT_GUI_QA_MAPPING_CAPTURE")) {
            if (auto * screen = QApplication::primaryScreen()) {
              screen->grabWindow(0).save(QString::fromLocal8Bit(path));
            }
          }
          QTimer::singleShot(0, [qaMap]() {
            auto * dialog = qobject_cast<QInputDialog *>(QApplication::activeModalWidget());
            if (dialog) {dialog->setTextValue(qaMap); dialog->accept();}
          });
          *qaStep = 11; save->click();
          std::fprintf(stderr, "GUI_WORKFLOW_SAVE\n");
        } else if (*qaStep == 11 && maps->currentText() == qaMap && stop->isEnabled()) {
          *qaStep = 12; stop->click(); std::fprintf(stderr, "GUI_WORKFLOW_STOP_MAPPING\n");
        } else if (*qaStep == 12 && scan->isEnabled() && start->isEnabled()) {
          *qaStep = 13; start->click(); std::fprintf(stderr, "GUI_WORKFLOW_START_FIRE\n");
        } else if (*qaStep == 13 && status->text().contains("POSE")) {
          if (!rviz) {return;}
          auto * manager = rviz->getManager()->getToolManager();
          for (int i = 0; i < manager->numTools(); ++i) {
            if (manager->getTool(i)->getClassId() == "rviz_default_plugins/SetInitialPose") {
              std::fprintf(stderr, "GUI_WORKFLOW_INITIAL_POSE_TOOL\n"); *qaStep = 131; break;
            }
          }
        } else if (*qaStep == 131 && stop->isEnabled() && status->text().contains("READY_GATE2")) {
          if (!rviz) {return;}
          auto * manager = rviz->getManager()->getToolManager();
          bool forbidden = false;
          for (int i = 0; i < manager->numTools(); ++i) {
            const auto id = manager->getTool(i)->getClassId();
            forbidden |= id == "rviz_default_plugins/SetInitialPose" || id == "rviz_default_plugins/SetGoal";
          }
          if (forbidden) {return;}
          std::fprintf(stderr, "GUI_WORKFLOW_UNARMED_GOAL_LOCKED\n");
          *qaStep = 14; stop->click(); std::fprintf(stderr, "GUI_WORKFLOW_STOP_FIRE\n");
        } else if (*qaStep == 14 && scan->isEnabled()) {
          if (*idleAt == std::chrono::steady_clock::time_point{}) {
            *idleAt = std::chrono::steady_clock::now();
            std::fprintf(stderr, "GUI_CYCLE_IDLE_ROS_CONTEXT=%d\n", rclcpp::ok() ? 1 : 0);
          }
          if (std::chrono::steady_clock::now() - *idleAt < std::chrono::seconds(1)) {return;}
          *idleAt = {};
          if (mode == "cycles" && ++*qaLoops < 3) {*qaStep = 0;}
          else {*qaStep = 15; qa->stop(); frame.close();}
        }
      });
      qa->start(30);
      QTimer::singleShot(mode == "cycles" ? 45000 : 15000, &app, [&app]() {std::fprintf(stderr, "GUI_FIXTURE_TIMEOUT\n"); app.exit(5);});
      QObject::connect(&app, &QCoreApplication::aboutToQuit, &frame, [qaStep, qaLoops, idleAt, qaState]() {
        if (!qaState.isEmpty()) {
          QFile saved(qaState);
          if (saved.open(QIODevice::WriteOnly | QIODevice::Truncate)) {saved.write(QByteArray::number(*qaStep) + " " + QByteArray::number(*qaLoops));}
        }
        delete qaLoops; delete idleAt;
        std::fprintf(stderr, "GUI_FIXTURE_EXIT_STEP=%d\n", *qaStep); delete qaStep;
      });
    }
#endif
    if (const char * capture = preview ? std::getenv("FIRE_ROBOT_PREVIEW_CAPTURE") : nullptr) {
      const QString path = QString::fromLocal8Bit(capture);
      bool ok = false;
      const int requestedDelay = qEnvironmentVariableIntValue("FIRE_ROBOT_PREVIEW_CAPTURE_DELAY_MS", &ok);
      const int delay = ok && requestedDelay >= 1000 && requestedDelay <= 15000 ? requestedDelay : 4000;
      QTimer::singleShot(delay, &frame, [&frame, path]() {
        auto * screen = QApplication::primaryScreen();
        const bool ok = screen && screen->grabWindow(frame.winId()).save(path);
        std::fprintf(stderr, "PREVIEW_CAPTURE=%s\n", ok ? "OK" : "FAILED");
        const auto * camera = frame.findChild<QWidget *>("camera_widget");
        const auto * task = frame.findChild<QLabel *>("operator_status");
        const auto * actuator = frame.findChild<QLabel *>("actuator_status");
        std::fprintf(stderr, "PREVIEW_FRAME=%s\n", camera && camera->property("frame_fresh").toBool() ? "FRESH" : "WAITING");
        if (camera) {std::fprintf(stderr, "PREVIEW_CAMERA_SIZE=%dx%d\n", camera->width(), camera->height());}
        if (task) {std::fprintf(stderr, "PREVIEW_TASK=%s\n", task->text().toUtf8().constData());}
        if (actuator) {std::fprintf(stderr, "PREVIEW_ACTUATOR=%s\n", actuator->text().toUtf8().constData());}
        frame.close();
      });
    }
    rc = app.exec();
    timer.stop();
    if (control) {
      detachView();
      if (mappingChild) {
        auto * child = mappingChild.data();
        QObject::disconnect(child, nullptr, &frame, nullptr);
        if (!child->waitForFinished(1500)) {child->kill(); child->waitForFinished(1500);}
        mappingChild = nullptr;
      }
    }
  }
  demo.reset();
  observer.reset();
  outer.reset();
  abstraction.reset();
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return rc;
}
