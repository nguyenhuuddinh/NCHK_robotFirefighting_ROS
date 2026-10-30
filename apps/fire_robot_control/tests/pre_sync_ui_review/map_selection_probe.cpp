#include "session_ui.hpp"
#include <QApplication>
#include <QAbstractItemView>
#include <QComboBox>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLocalServer>
#include <QLocalSocket>
#include <QPushButton>
#include <QSignalSpy>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

int main(int argc, char **argv)
{
  if (argc != 2 || qgetenv("ROS_DOMAIN_ID") != "211" || qgetenv("ROS_LOCALHOST_ONLY") != "1") {return 2;}
  const QString config = QString::fromLocal8Bit(argv[1]);
  int one = 1;
  rclcpp::init(one, argv);
  QApplication app(one, argv);
  auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_map_selection_qa");
  {
    rviz_common::VisualizationFrame frame(abstraction);
    frame.setApp(&app); frame.setSplashPath(""); frame.initialize(abstraction, config);
    QTemporaryDir dir;
    QLocalServer server;
    if (!dir.isValid() || !server.listen(dir.path() + "/control.sock")) {return 3;}
    FrontendBootstrap boot{"review", "review", "review", QString(64, 'a'), dir.path() + "/control.sock"};
    AppClient client(boot);
    QElapsedTimer wait; wait.start();
    while (!server.hasPendingConnections() && wait.elapsed() < 1000) {app.processEvents(); QThread::msleep(1);}
    std::unique_ptr<QLocalSocket> peer(server.nextPendingConnection());
    if (!peer) {return 3;}
    app.processEvents();
    QComboBox maps(&frame);
    maps.setGeometry(40, 40, 240, 35);
    QPushButton scan(&frame), save(&frame), start(&frame), cancel(&frame), stop(&frame);
    QLabel status(&frame);
    SessionUi ui(frame, client, &maps, &scan, &save, &start, &cancel, &stop, &status, [](bool) {});
    QJsonArray catalog{
      QJsonObject{{"name", "map_one"}, {"yaml_path", "/tmp/map_one.yaml"}},
      QJsonObject{{"name", "map_two"}, {"yaml_path", "/tmp/map_two.yaml"}}};
    QJsonObject row{{"event", "STATUS"}, {"phase", "IDLE"}, {"stage", "IDLE"},
      {"session", QJsonValue()}, {"maps", catalog}, {"selected", QJsonValue()}};
    client.message(row);
    frame.show(); maps.show(); maps.raise();
    QTest::qWait(100);
    QSignalSpy resets(maps.model(), &QAbstractItemModel::modelReset);
    QTimer refresh;
    QObject::connect(&refresh, &QTimer::timeout, &app, [&]() {client.message(row);});
    refresh.start(100);
    maps.showPopup();
    maps.view()->setCurrentIndex(maps.model()->index(2, 0));
    QTest::qWait(450);
    QJsonObject results{{"idle_enabled", maps.isEnabled()}, {"count", maps.count()},
      {"unchanged_model_resets", resets.count()}, {"popup_visible", maps.view()->isVisible()},
      {"highlight_after_refresh", maps.view()->currentIndex().row()}};
    refresh.stop();
    QSignalSpy changes(&maps, QOverload<int>::of(&QComboBox::currentIndexChanged));
    const auto target = maps.model()->index(2, 0);
    const auto point = maps.view()->visualRect(target).center();
    QTest::mouseClick(maps.view()->viewport(), Qt::LeftButton, Qt::NoModifier, point);
    QTest::qWait(30);
    results["mouse_selected_path"] = maps.currentData().toJsonObject().value("yaml_path");
    results["mouse_change_count"] = changes.count();
    // Acknowledge the selected path and keep receiving periodic statuses.
    row["selected"] = "/tmp/map_two.yaml";
    for (int i = 0; i < 5; ++i) {client.message(row);}
    results["ack_selected_path"] = maps.currentData().toJsonObject().value("yaml_path");
    results["unchanged_model_resets_after_ack"] = resets.count();
    catalog.append(QJsonObject{{"name", "map_new"}, {"yaml_path", "/tmp/map_new.yaml"}});
    row["maps"] = catalog;
    row["selected"] = "/tmp/map_new.yaml";
    client.message(row);
    results["new_catalog_count"] = maps.count();
    results["new_catalog_selected_path"] = maps.currentData().toJsonObject().value("yaml_path");
    QTest::qWait(30);
    const auto bytes = peer->readAll();
    int selects = 0, starts = 0;
    QString selected;
    for (const auto &line : bytes.split('\n')) {
      const auto message = QJsonDocument::fromJson(line).object();
      if (message.value("event").toString() == "SELECT") {++selects; selected = message.value("argument").toString();}
      if (message.value("event").toString() == "SCAN" || message.value("event").toString() == "FIRE") {++starts;}
    }
    results["select_commands"] = selects;
    results["select_command_path"] = selected;
    results["start_commands"] = starts;
    std::printf("MAP_SELECTION_QA %s\n", QJsonDocument(results).toJson(QJsonDocument::Compact).constData());
  }
  rclcpp::shutdown();
  return 0;
}
