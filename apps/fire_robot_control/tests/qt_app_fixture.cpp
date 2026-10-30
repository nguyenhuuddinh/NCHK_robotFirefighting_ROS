// Local IPC end-to-end fixture. It has no ROS, RViz or robot publisher.
#include "../native_rviz/app_client.hpp"
#include <QCoreApplication>
#include <QJsonArray>
#include <QTimer>
#include <cstdio>
#include <unistd.h>

int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  if (argc != 4 || QByteArray(argv[1]) != "--control") {return 2;}
  bool ok = false;
  int fd = QByteArray(argv[3]).toInt(&ok);
  if (!ok) {return 2;}
  auto bootstrap = readFrontendBootstrap(fd);
  ::close(fd);
  if (!bootstrap) {return 2;}
  AppClient client(*bootstrap);
  const QByteArray mode = qgetenv("FIRE_ROBOT_QA_FIXTURE");
  bool selected = false, started = false, finished = false;
  QObject::connect(&client, &AppClient::lost, &app, [&]() {app.exit(3);});
  QObject::connect(&client, &AppClient::message, &app, [&](const QJsonObject & row) {
    const auto event = row.value("event").toString();
    if (event == "STATUS" && row.value("phase").toString() == "IDLE" && !started) {
      if (mode == "scan") {
        started = client.send("SCAN");
      } else if ((mode == "fire" || mode == "handshake") && !selected && !row.value("maps").toArray().isEmpty()) {
        const auto map = row.value("maps").toArray()[0].toObject();
        selected = client.send("SELECT", map.value("yaml_path"));
      } else if ((mode == "fire" || mode == "handshake") && selected && !row.value("selected").toString().isEmpty()) {
        started = client.send("FIRE");
      }
    }
    if (mode == "handshake" && event == "STATUS" && row.value("phase").toString() == "RUNNING" &&
      client.operatorConnected()) {
      std::fprintf(stdout, "FIXTURE_OPERATOR_CONNECTED\n");
      client.stop();
    }
    if (event == "FINISHED") {
      finished = true;
      std::fprintf(stdout, "FIXTURE_FINISHED stopped=%d clean=%d started=%d error=%s\n",
        row.value("robot_stop_confirmed").isNull() ? -1 : row.value("robot_stop_confirmed").toBool(),
        row.value("cleanup_clean").toBool(), row.value("session_was_started").toBool(),
        row.value("error").toString().toUtf8().constData());
      client.stop(true);
    }
    if (event == "STATUS" && row.value("close_ready").toBool() && finished) {
      std::fprintf(stdout, "FIXTURE_CLOSE_READY\n");
      app.quit();
    }
  });
  QTimer::singleShot(12000, &app, [&]() {app.exit(4);});
  return app.exec();
}
