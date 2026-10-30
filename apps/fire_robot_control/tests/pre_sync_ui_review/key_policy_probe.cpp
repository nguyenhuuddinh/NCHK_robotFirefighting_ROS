// Actual Qt key dispatch and RViz tools; localhost only, no robot telemetry.
#define main production_entry_unused_in_key_qa
#include "../../native_rviz/main.cpp"
#undef main
#include <QDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QLocalServer>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QTest>
#include <QToolButton>
#include <rviz_common/render_panel.hpp>

int main(int argc, char **argv)
{
  if (argc != 2 || qgetenv("ROS_DOMAIN_ID") != "211" || qgetenv("ROS_LOCALHOST_ONLY") != "1") {return 2;}
  const QString config = QString::fromLocal8Bit(argv[1]);
  int one = 1;
  QApplication app(one, argv);
  rclcpp::init(one, argv);
  QJsonObject result;
  {
    QTemporaryDir temp; QLocalServer server;
    if (!server.listen(temp.path()+"/keys.sock")) {return 3;}
    FrontendBootstrap boot{"key-qa","generation","nonce",QString(64,'a'),temp.path()+"/keys.sock"};
    AppClient client(boot); QTest::qWait(30);
    std::unique_ptr<QLocalSocket> peer(server.nextPendingConnection());
    if (!peer) {return 4;}
    QMainWindow frame; std::unique_ptr<Observer> observer;
    auto *ui = configureUi(frame, nullptr, observer, config, false, &client);
    auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_key_qa");
    auto *view = new rviz_common::VisualizationFrame(abstraction);
    view->setApp(&app); view->setSplashPath(""); view->initialize(abstraction, config);
    frame.setCentralWidget(view); ui->setView(view); frame.resize(1300,800); frame.show();
    // This IPC fixture supplies permission explicitly below. The real observer
    // is absent; stop its UI refresh from revoking the synthetic test grant.
    // AppClient's heartbeat is a member of client, not a child of the frame.
    for (auto *timer : frame.findChildren<QTimer*>()) {timer->stop();}
    auto *start = frame.findChild<QPushButton*>("start_button");
    auto *scan = frame.findChild<QPushButton*>("scan_button");
    auto *stop = frame.findChild<QPushButton*>("stop_button");
    auto *save = frame.findChild<QPushButton*>("save_button");
    auto *manager = view->getManager()->getToolManager();
    auto *panel = view->getManager()->getRenderPanel();
    QJsonObject row{{"event","STATUS"},{"phase","IDLE"},{"stage","IDLE"},
      {"maps",QJsonArray()},{"session",QJsonValue()},{"selected","/fake/map.yaml"}};
    int session = 0;
    auto commands = [&]() {
      QTest::qWait(35);
      QJsonArray messages;
      for (const auto &line : peer->readAll().split('\n')) {
        const auto m = QJsonDocument::fromJson(line).object();
        if (!m.isEmpty() && m.value("event") != "HEARTBEAT") {messages.append(m);}
      }
      return messages;
    };
    auto hasStop = [](const QJsonArray &messages) {
      for (const auto &m : messages) {
        const auto event = m.toObject().value("event").toString();
        if (event == "STOP" || event == "CLOSE") {return true;}
      }
      return false;
    };
    auto begin = [&](bool mapping) {
      row["phase"] = "IDLE"; row["stage"] = "IDLE"; row["session"] = QJsonValue(); client.message(row);
      if (mapping) {scan->click();} else {start->click();}
      const auto id = QString("key-session-%1").arg(++session);
      client.message(QJsonObject{{"event","NEW_SESSION"},{"session",id},{"kind",mapping?"mapping":"fire"}});
      row["phase"] = mapping?"MAPPING":"STARTING"; row["stage"] = mapping?"MAPPING":"POSE";
      row["session"] = id; client.message(row); QTest::qWait(50); commands();
      frame.activateWindow(); panel->setFocus(); QTest::qWait(30);
    };
    for (const auto id : {QString("rviz_default_plugins/SetInitialPose"), QString("rviz_default_plugins/SetGoal")}) {
      const auto name = id.endsWith("SetGoal") ? QString("goal") : QString("pose");
      begin(false);
      if (name == "goal") {
        row["phase"] = "RUNNING"; row["stage"] = "READY_GATE2"; client.message(row); ui->updateGoalPermission(true);
      }
      rviz_common::Tool *tool = nullptr;
      for (int i=0; i<manager->numTools(); ++i) {if (manager->getTool(i)->getClassId() == id) {tool = manager->getTool(i);}}
      if (!tool) {return 5;}
      manager->setCurrentTool(tool); panel->setFocus(); QTest::qWait(20); commands();
      QTest::keyClick(panel, Qt::Key_Escape);
      const auto messages = commands();
      result[name+"_escape_sends_no_stop"] = !hasStop(messages);
      result[name+"_escape_returns_to_default_tool"] = manager->getCurrentTool() == manager->getDefaultTool();
      result[name+"_escape_keeps_session_and_stop_button"] = ui->viewAllowed() && stop->isEnabled();
      const auto *warning = frame.findChild<QLabel*>("warning");
      result[name+"_escape_does_not_enter_stopping"] = !warning->text().contains("STOPPING");
    }
    begin(false);
    row["phase"] = "RUNNING"; row["stage"] = "READY_GATE2"; client.message(row); ui->updateGoalPermission(true);
    rviz_common::Tool *goalTool = nullptr;
    for (int i=0; i<manager->numTools(); ++i) {
      if (manager->getTool(i)->getClassId() == "rviz_default_plugins/SetGoal") {goalTool = manager->getTool(i);}
    }
    QToolButton *goalButton = nullptr;
    for (auto *button : view->findChildren<QToolButton*>()) {
      if (button->text() == goalTool->getName()) {goalButton = button;}
    }
    result["goal_toolbar_button_found"] = goalButton != nullptr;
    if (goalButton) {
      manager->setCurrentTool(manager->getDefaultTool());
      panel->setFocus(); QTest::mouseClick(goalButton, Qt::LeftButton); QTest::qWait(30); commands();
      result["goal_toolbar_click_activates_goal"] = manager->getCurrentTool() == goalTool;
      auto *focus = QApplication::focusWidget();
      QTest::keyClick(focus ? focus : goalButton, Qt::Key_Escape);
      result["goal_toolbar_escape_sends_no_stop"] = !hasStop(commands()) && ui->viewAllowed();
      result["goal_toolbar_escape_returns_to_default_tool"] = manager->getCurrentTool() == manager->getDefaultTool();
    }
    begin(false);
    QTest::keyClick(panel, Qt::Key_Escape); QTest::keyClick(panel, Qt::Key_Escape);
    result["escape_on_default_keeps_session"] = !hasStop(commands()) && ui->viewAllowed();
    QTest::keyClick(panel, Qt::Key_Q);
    result["q_still_sends_stop_from_rviz"] = hasStop(commands()) && !ui->viewAllowed();
    begin(false);
    result["stop_label_only_q"] = stop->text().contains("Q") && !stop->text().contains("Esc");
    stop->click();
    result["stop_button_still_sends_stop"] = hasStop(commands());

    begin(true);
    QDialog *keyboard = nullptr;
    for (auto *dialog : frame.findChildren<QDialog*>()) {
      if (dialog->windowTitle().contains(QString::fromUtf8("Bàn phím"))) {keyboard = dialog;}
    }
    if (!keyboard) {return 6;}
    keyboard->activateWindow(); keyboard->setFocus(); QTest::qWait(20); commands();
    QTest::keyClick(keyboard, Qt::Key_Escape);
    result["keyboard_escape_no_stop_and_stays_open"] = !hasStop(commands()) && keyboard->isVisible();
    QTest::keyClick(keyboard, Qt::Key_Q);
    result["keyboard_q_still_stops"] = hasStop(commands());
    // Esc in a text-entry dialog keeps normal Qt cancel semantics.
    begin(true);
    bool rejected = false;
    QTimer::singleShot(40, &frame, [&]() {
      if (auto *dialog = qobject_cast<QInputDialog*>(QApplication::activeModalWidget())) {
        QTest::keyClick(dialog, Qt::Key_Escape); rejected = true;
      }
    });
    commands(); save->click();
    result["save_dialog_escape_cancels_without_stop"] = rejected && !hasStop(commands());

    // The mapping child has a process-wide key filter. Check actual Qt dispatch
    // without treating stdout markers as any robot STOP proof.
    QTemporaryFile output; if (!output.open()) {return 7;}
    std::fflush(stdout); const int saved = ::dup(STDOUT_FILENO);
    ::dup2(output.handle(), STDOUT_FILENO);
    MappingViewKeys keys; QWidget child;
    child.show(); child.activateWindow(); child.setFocus(); QTest::qWait(40);
    app.installEventFilter(&keys);
    QTest::keyClick(&child, Qt::Key_Escape);
    std::fflush(stdout);
    QFile escapedFile(output.fileName()); escapedFile.open(QIODevice::ReadOnly); const auto escaped = escapedFile.readAll();
    QTest::keyClick(&child, Qt::Key_Q);
    std::fflush(stdout);
    QFile stoppedFile(output.fileName()); stoppedFile.open(QIODevice::ReadOnly); const auto stopped = stoppedFile.readAll();
    app.removeEventFilter(&keys); ::dup2(saved, STDOUT_FILENO); ::close(saved);
    result["mapping_child_escape_sends_no_stop"] = !escaped.contains("APP_CHILD_STOP");
    result["mapping_child_q_still_sends_stop"] = stopped.count("APP_CHILD_STOP") == 1;
    row["phase"] = "IDLE"; row["session"] = QJsonValue(); row["close_ready"] = true; client.message(row);
    ui->setView(nullptr); QTest::qWait(30);
  }
  rclcpp::shutdown();
  std::puts(("QA_KEY_POLICY "+QJsonDocument(result).toJson(QJsonDocument::Compact)).constData());
  for (auto it=result.begin(); it!=result.end(); ++it) {if (!it.value().toBool()) {return 1;}}
  return 0;
}
