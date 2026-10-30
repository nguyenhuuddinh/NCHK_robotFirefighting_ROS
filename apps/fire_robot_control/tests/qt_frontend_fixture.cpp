#include "../native_rviz/frontend_client.hpp"
#include "../native_rviz/frontend_bootstrap.hpp"

#include <QCoreApplication>
#include <QThread>
#include <QTimer>
#include <unistd.h>

// Local test fixture only; it does not connect to ROS or real hardware.
int main(int argc, char ** argv)
{
  QCoreApplication app(argc, argv);
  if (argc != 4) {return 2;}
  const auto mode = QByteArray(argv[2]);
  bool validFd = false;
  const int fd = QString::fromLocal8Bit(argv[3]).toInt(&validFd);
  if (!validFd) {return 2;}
  const auto bootstrap = readFrontendBootstrap(fd);
  ::close(fd);
  if (!bootstrap || bootstrap->socketPath != QString::fromLocal8Bit(argv[1])) {return 2;}
  FrontendClient client(bootstrap->session, bootstrap->generation,
    mode == "wrong-nonce" ? "wrong" : bootstrap->nonce);
  client.connectTo(QString::fromLocal8Bit(argv[1]));
  QTimer wait;
  wait.setInterval(10);
  QObject::connect(&wait, &QTimer::timeout, &app, [&]() {
    if (!client.connected()) {return;}
    wait.stop();
    if (mode == "actions") {
      QTimer::singleShot(80, &app, [&]() {client.sendCancel();});
      QTimer::singleShot(180, &app, [&]() {client.sendStop();});
    } else if (mode == "freeze") {
      QTimer::singleShot(150, &app, []() {QThread::msleep(800);});
    } else if (mode == "disconnect") {
      QTimer::singleShot(150, &app, &QCoreApplication::quit);
    } else if (mode == "wrong-nonce") {
      // Keep sending heartbeats under the wrong identity.
    } else if (mode == "alive") {
      // Keep the actual Qt event-loop heartbeat running for native-block QA.
    } else {app.exit(2);}
  });
  wait.start();
  QTimer::singleShot(2500, &app, &QCoreApplication::quit);
  return app.exec();
}
