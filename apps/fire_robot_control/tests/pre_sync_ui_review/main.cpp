#include "session_ui.hpp"
#include <QApplication>
#include <QComboBox>
#include <QElapsedTimer>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLocalServer>
#include <QLocalSocket>
#include <QPushButton>
#include <QStatusBar>
#include <QTemporaryDir>
#include <QThread>
#include <QTest>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/tool.hpp>

int main(int argc, char **argv)
{
  if (argc != 2 || qgetenv("ROS_DOMAIN_ID") != "211" || qgetenv("ROS_LOCALHOST_ONLY") != "1") {return 2;}
  const QString config = QString::fromLocal8Bit(argv[1]);
  int one = 1;
  rclcpp::init(one, argv);
  QApplication app(one, argv);
  auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_ui_review");
  {
    rviz_common::VisualizationFrame frame(abstraction);
    frame.setApp(&app); frame.setSplashPath(""); frame.initialize(abstraction, config);
    auto * manager = frame.getManager()->getToolManager();
    const auto has = [manager](const QString &id) {
      for (int i = 0; i < manager->numTools(); ++i) {if (manager->getTool(i)->getClassId() == id) {return true;}}
      return false;
    };
    const QString pose = "rviz_default_plugins/SetInitialPose", goal = "rviz_default_plugins/SetGoal";
    QJsonObject results;
    // Actual inert local control connection; only status/button events are
    // simulated. No robot runner, canvas clicks, Pose or Goal publications.
    for (int scene = 0; scene < 3; ++scene) {
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
      QPushButton scan(&frame), save(&frame), start(&frame), cancel(&frame), stop(&frame);
      QLabel status(&frame);
      SessionUi ui(frame, client, &maps, &scan, &save, &start, &cancel, &stop, &status, [](bool) {});
      QJsonObject row{{"event", "STATUS"}, {"phase", "IDLE"}, {"stage", "IDLE"},
        {"session", QJsonValue()}, {"maps", QJsonArray()}, {"selected", "/tmp/inert-review.yaml"}};
      const auto begin = [&](const QString &id) {
        row["phase"] = "IDLE"; row["stage"] = "IDLE"; row["session"] = QJsonValue();
        client.message(row); start.click();
        results["start_feedback_immediate"] = !start.isEnabled()
          && start.text().contains(QString::fromUtf8("Đang yêu cầu"))
          && frame.statusBar()->currentMessage().contains(QString::fromUtf8("Đã gửi yêu cầu"));
        client.message(QJsonObject{{"event", "NEW_SESSION"}, {"session", id}, {"kind", "fire"}});
        row["phase"] = "STARTING"; row["stage"] = "POSE"; row["session"] = id;
        client.message(row);
      };
      begin("first");
      if (!has(pose)) {return 4;}
      if (scene == 0) {
        results["pose_before"] = has(pose);
        stop.click(); results["pose_after_stop"] = has(pose);
        results["stop_feedback_immediate"] = stop.text().contains(QString::fromUtf8("Đang yêu cầu dừng"));
        client.message(row); results["pose_after_late_status"] = has(pose);
        begin("second"); results["pose_after_explicit_restart"] = has(pose);
        row["phase"] = "RUNNING"; row["stage"] = "READY_GATE2";
        client.message(row); ui.updateGoalPermission(true);
        results["goal_before"] = has(goal);
        stop.click();
        client.message(row); ui.updateGoalPermission(true);
        results["goal_after_stop_refresh"] = has(goal);
        client.message(QJsonObject{{"event", "NEW_SESSION"}, {"session", "unsolicited"}, {"kind", "fire"}});
        row["phase"] = "STARTING"; row["stage"] = "POSE"; row["session"] = "unsolicited";
        client.message(row); results["pose_after_unsolicited_session"] = has(pose);
        begin("third"); results["pose_after_second_restart"] = has(pose);
        row["session"] = "foreign"; client.message(row);
        results["pose_for_foreign_status"] = has(pose);
        row["session"] = "third"; row["phase"] = "MAPPING"; client.message(row);
        results["pose_in_mapping"] = has(pose);
        row["phase"] = "STARTING"; row["stage"] = "PREFLIGHT";
        row["last_stop"] = QJsonObject{{"session_was_started", true}, {"robot_stop_confirmed", true}, {"cleanup_clean", true}};
        client.message(row);
        results["old_stop_label_in_new_session"] = status.text().contains(QString::fromUtf8("ĐÃ XÁC NHẬN"));
        row["phase"] = "IDLE"; row["stage"] = "IDLE"; row["session"] = QJsonValue();
        client.message(row); start.click();
        // STOP before the start acknowledgement dominates that late ack.
        row["phase"] = "STARTING"; row["stage"] = "PREFLIGHT"; client.message(row);
        stop.click();
        client.message(QJsonObject{{"event", "NEW_SESSION"}, {"session", "late"}, {"kind", "fire"}});
        row["phase"] = "STARTING"; row["stage"] = "POSE"; row["session"] = "late";
        client.message(row); results["pose_after_stop_before_ack"] = has(pose);
        row["phase"] = "BLOCKED"; row["stage"] = "BLOCKED"; row["session"] = QJsonValue();
        client.message(row);
        results["retry_button_enabled"] = scan.isEnabled() && scan.text() == QString::fromUtf8("Kiểm tra lại");
        app.processEvents(); peer->readAll();
        // A human holds the mouse across several 100 ms STATUS refreshes.
        // Disabling/re-enabling an enabled button mid-press cancels clicked().
        QTest::mousePress(&scan, Qt::LeftButton);
        for (int i = 0; i < 4; ++i) {client.message(row); QTest::qWait(30);}
        QTest::mouseRelease(&scan, Qt::LeftButton); app.processEvents();
        peer->waitForReadyRead(30);
        int heldChecks = 0;
        for (const auto & line : peer->readAll().split('\n')) {
          heldChecks += QJsonDocument::fromJson(line).object().value("event").toString() == "RECHECK";
        }
        results["held_retry_survives_status_refresh"] = heldChecks == 1;
        client.message(row); app.processEvents(); peer->readAll();
        scan.click(); scan.click(); app.processEvents();
        peer->waitForReadyRead(30);
        const auto commands = peer->readAll().split('\n');
        int checks = 0, starts = 0;
        for (const auto & line : commands) {
          const auto message = QJsonDocument::fromJson(line).object();
          const auto event = message.value("event").toString();
          checks += event == "RECHECK"; starts += event == "SCAN" || event == "FIRE";
        }
        results["retry_one_check_no_start"] = checks == 1 && starts == 0;
        row["phase"] = "CHECKING"; row["stage"] = "CHECKING"; client.message(row);
        ui.updateGoalPermission(true);
        results["checking_all_commands_locked"] = !scan.isEnabled() && !start.isEnabled() && !maps.isEnabled()
          && !save.isEnabled() && !has(pose) && !has(goal);
        row["phase"] = "BLOCKED"; row["stage"] = "BLOCKED";
        row["recovery_attempt"] = 1; row["recovery_result"] = "FAILED";
        row["error"] = "RECOVERY_PI_SSH_FAILED";
        row["recovery_message"] = QString::fromUtf8("Chưa kết nối được Pi"); client.message(row);
        results["retry_failure_visible_with_reason"] = status.text().contains(QString::fromUtf8("Kiểm tra lần 1: CHƯA ĐẠT"))
          && status.text().contains("RECOVERY_PI_SSH_FAILED") && scan.isEnabled() && !start.isEnabled();
        // No CHECKING status between these snapshots: both failures must
        // remain distinguishable even when the audit completes immediately.
        row["recovery_attempt"] = 2;
        for (int i = 0; i < 4; ++i) {client.message(row);}
        results["retry_fast_failure_count_persists"] = status.text().contains(QString::fromUtf8("Kiểm tra lần 2: CHƯA ĐẠT"));
        row["error"] = QJsonValue(); row["recovery_attempt"] = 3; row["recovery_result"] = "SUCCEEDED";
        row["phase"] = "IDLE"; row["stage"] = "IDLE"; row["last_stop"] = QJsonValue();
        row["recovery_message"] = "Recovered"; client.message(row);
        results["recovered_idle_no_tools_or_old_stop"] = scan.isEnabled()
          && scan.text() == QString::fromUtf8("Quét map") && !has(pose) && !has(goal)
          && !status.text().contains(QString::fromUtf8("ĐÃ XÁC NHẬN"));
        results["retry_success_visible"] = status.text().contains(QString::fromUtf8("Kiểm tra lần 3: THÀNH CÔNG"));
        row["phase"] = "MAPPING"; row["save_pending"] = true; client.message(row);
        results["save_busy_label"] = save.text() == QString::fromUtf8("Đang lưu map…") && !save.isEnabled();
        row["phase"] = "STOPPING"; client.message(row);
        results["stop_busy_label"] = stop.text() == QString::fromUtf8("Đang dừng phiên…");
      } else if (scene == 1) {
        client.lost(); results["pose_after_lost"] = has(pose);
        client.message(row); results["pose_after_lost_late_status"] = has(pose);
        client.message(QJsonObject{{"event", "NEW_SESSION"}, {"session", "after-loss"}, {"kind", "fire"}});
        row["session"] = "after-loss"; client.message(row);
        results["pose_after_lost_new_session"] = has(pose);
      } else {
        row["phase"] = "RUNNING"; row["stage"] = "READY_GATE2";
        client.message(row); ui.updateGoalPermission(true);
        if (!has(goal)) {return 4;}
        client.lost(); results["goal_on_lost"] = has(goal);
        ui.updateGoalPermission(true); results["goal_after_refresh"] = has(goal);
      }
    }
    std::printf("UI_REVIEW %s\n", QJsonDocument(results).toJson(QJsonDocument::Compact).constData());
  }
  rclcpp::shutdown();
  return 0;
}
