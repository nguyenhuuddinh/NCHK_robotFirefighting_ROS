#include "../native_rviz/frontend_client.hpp"
#include "../native_rviz/frontend_bootstrap.hpp"
#include "../native_rviz/strict_json.hpp"
#include "../native_rviz/goal_admission.hpp"
#include "../native_rviz/navigation_recovery_label.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QLocalServer>
#include <QTemporaryDir>
#include <QTemporaryFile>
#include <QTest>
#include <QThread>
#include <poll.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

class FrontendTest : public QObject
{
  Q_OBJECT
private slots:
  void eventsAndStopPriority();
  void heartbeatUsesUiEventLoop();
  void queueOverflowDisconnects();
  void bootstrapRequiresPrivateReadOnlyDescriptor();
  void stopBeforeConnectionStillDeliversQ();
  void rejectsAmbiguousAndInvalidReports();
  void goalRequiresCurrentAppAdmission();
  void navigationHoldShowsRecoveryWithoutGrantingGoal();
};

void FrontendTest::navigationHoldShowsRecoveryWithoutGrantingGoal()
{
  QVERIFY(navigationRecoveryLabel({}).isEmpty());
  for (const auto & state : {"WAIT_COMMAND", "WAIT_STOP", "VERIFYING", "REACQUIRING", "BLOCKED"}) {
    QJsonObject recovery{{"state", state}, {"reason", "NAV_COMMAND_GAP"}};
    QVERIFY(!navigationRecoveryLabel(recovery).isEmpty());
    QJsonObject report{{"app_navigation_recovery", recovery}};
    QVERIFY(!appGoalAllowed(report, .1));
  }
  QVERIFY(navigationRecoveryLabel({{"state", "RESUMED"}}).isEmpty());
}

void FrontendTest::goalRequiresCurrentAppAdmission()
{
  auto report = QJsonDocument::fromJson(R"({"schema_version":1,"scope":"R5_SESSION_INTENT_ONLY",
    "armed":true,"closed":false,"automatic":{"enabled":true},"fault":null,
    "cycle":null,"phase":"ARMED_IDLE","app_goal_admission":{"allowed":true}})").object();
  QVERIFY(appGoalAllowed(report, .1));
  QVERIFY(!appGoalAllowed(report, .501));
  QVERIFY(!appGoalAllowed(report, -.1));
  QVERIFY(!appGoalAllowed(report, std::nan("")));
  for (const auto & key : {"input_hold", "identity_hold", "graph_hold", "closed"}) {
    auto held = report; held[key] = true;
    QVERIFY(!appGoalAllowed(held, .1));
  }
  auto changed = report; changed.remove("app_goal_admission");
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["app_goal_admission"] = QJsonObject{{"allowed", false}};
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["phase"] = "CANCEL_WAIT_CLEAR";
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["cycle"] = QJsonObject{{"cycle_id", "active"}};
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["fault"] = "BOOT_OR_SERIAL_EPOCH_CHANGED";
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["armed"] = false;
  QVERIFY(!appGoalAllowed(changed, .1));
  changed = report; changed["runtime_rebind"] = QJsonObject{{"held", true}};
  QVERIFY(!appGoalAllowed(changed, .1));
}

void FrontendTest::eventsAndStopPriority()
{
  QTemporaryDir dir;
  QVERIFY(dir.isValid());
  QLocalServer server;
  const auto path = dir.path() + "/socket";
  QVERIFY(server.listen(path));
  FrontendClient client("session", "generation", "nonce");
  QVERIFY(!client.sendCancel());
  client.connectTo(path);
  QTRY_VERIFY_WITH_TIMEOUT(server.hasPendingConnections(), 2000);
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  QTRY_VERIFY_WITH_TIMEOUT(client.connected(), 2000);
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  auto first = QJsonDocument::fromJson(peer->readLine()).object();
  QCOMPARE(first.value("event").toString(), QString("HEARTBEAT"));
  QCOMPARE(first.value("session").toString(), QString("session"));
  QCOMPARE(first.value("generation").toString(), QString("generation"));
  QCOMPARE(first.value("seq").toInt(), 1);
  QVERIFY(first.value("sent_mono").toDouble() > 0);
  QVERIFY(client.sendCancel());
  QVERIFY(client.sendStop());
  QVERIFY(!client.sendCancel());
  QVERIFY(!client.sendStop());
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  auto cancel = QJsonDocument::fromJson(peer->readLine()).object();
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  auto stop = QJsonDocument::fromJson(peer->readLine()).object();
  QCOMPARE(cancel.value("event").toString(), QString("X"));
  QCOMPARE(stop.value("event").toString(), QString("Q"));
  QCOMPARE(cancel.value("seq").toInt(), 2);
  QCOMPARE(stop.value("seq").toInt(), 3);
}

void FrontendTest::heartbeatUsesUiEventLoop()
{
  QTemporaryDir dir;
  QLocalServer server;
  const auto path = dir.path() + "/socket";
  QVERIFY(server.listen(path));
  FrontendClient client("s", "g", "n");
  client.connectTo(path);
  QTRY_VERIFY_WITH_TIMEOUT(server.hasPendingConnections(), 2000);
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  peer->readAll();
  // Blocking this test's Qt thread simulates an unresponsive UI event loop.
  QThread::msleep(400);
  struct pollfd fd{static_cast<int>(peer->socketDescriptor()), POLLIN, 0};
  QCOMPARE(::poll(&fd, 1, 0), 0);
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 500);
}

void FrontendTest::queueOverflowDisconnects()
{
  QTemporaryDir dir;
  QLocalServer server;
  const auto path = dir.path() + "/socket";
  QVERIFY(server.listen(path));
  FrontendClient client("s", "g", "n");
  client.connectTo(path);
  QTRY_VERIFY_WITH_TIMEOUT(server.hasPendingConnections(), 2000);
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  QTRY_VERIFY_WITH_TIMEOUT(client.connected(), 2000);
  bool refused = false;
  for (int i = 0; i < 50000; ++i) {
    if (!client.sendCancel()) {refused = true; break;}
  }
  QVERIFY(refused);
  QVERIFY(!client.connected());
}

void FrontendTest::bootstrapRequiresPrivateReadOnlyDescriptor()
{
  QTemporaryDir dir;
  QTemporaryFile file(dir.path() + "/bootstrapXXXXXX");
  QVERIFY(file.open());
  const QByteArray session("8518a120-a012-4db3-bbf3-f73f67aed61e");
  const QByteArray generation(32, 'b'), nonce(32, 'c'), packet(64, 'a');
  const QByteArray socket = (dir.path() + "/robot.sock").toUtf8();
  const QByteArray content = "1\n" + session + "\n" + generation + "\n" + nonce +
    "\n" + packet + "\n" + socket.toBase64() + "\n";
  QCOMPARE(file.write(content), static_cast<qint64>(content.size()));
  QVERIFY(file.flush());
  QVERIFY(::chmod(file.fileName().toLocal8Bit().constData(), 0600) == 0);
  QVERIFY(!readFrontendBootstrap(file.handle()));  // writable FD is forbidden
  const int fd = ::open(file.fileName().toLocal8Bit().constData(), O_RDONLY);
  QVERIFY(fd >= 3);
  QVERIFY(::lseek(fd, 0, SEEK_END) >= 0);
  auto bootstrap = readFrontendBootstrap(fd);
  QVERIFY(bootstrap.has_value());
  QCOMPARE(bootstrap->session.toUtf8(), session);
  QCOMPARE(bootstrap->nonce.toUtf8(), nonce);
  QCOMPARE(bootstrap->socketPath.toUtf8(), socket);
  QVERIFY(::chmod(file.fileName().toLocal8Bit().constData(), 0644) == 0);
  QVERIFY(!readFrontendBootstrap(fd));
  ::close(fd);
}

void FrontendTest::stopBeforeConnectionStillDeliversQ()
{
  QTemporaryDir dir;
  QLocalServer server;
  const auto path = dir.path() + "/socket";
  QVERIFY(server.listen(path));
  FrontendClient client("s", "g", "n");
  client.connectTo(path);
  // Some Qt versions complete a local connection synchronously; both an
  // immediate send and a queued stop must deliver heartbeat before Q.
  client.sendStop();
  QTRY_VERIFY_WITH_TIMEOUT(server.hasPendingConnections(), 2000);
  QScopedPointer<QLocalSocket> peer(server.nextPendingConnection());
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  QCOMPARE(QJsonDocument::fromJson(peer->readLine()).object().value("event").toString(), QString("HEARTBEAT"));
  QTRY_VERIFY_WITH_TIMEOUT(peer->canReadLine(), 2000);
  QCOMPARE(QJsonDocument::fromJson(peer->readLine()).object().value("event").toString(), QString("Q"));
  QVERIFY(!client.sendCancel());
}

void FrontendTest::rejectsAmbiguousAndInvalidReports()
{
  QVERIFY(strictJsonObject(R"({"schema_version":1,"task":{"stage":"NAVIGATION"}})").has_value());
  QVERIFY(!strictJsonObject(R"({"armed":true,"armed":false})").has_value());
  QVERIFY(!strictJsonObject(R"({"nested":{"x":1,"\u0078":2}})").has_value());
  QVERIFY(!strictJsonObject(R"({"number":NaN})").has_value());
  QVERIFY(!strictJsonObject(QByteArray("{\"bad\":\"") + QByteArray(1, char(0xff)) + "\"}").has_value());
  QVERIFY(!strictJsonObject(QByteArray(256 * 1024 + 1, 'a')).has_value());
}

QTEST_GUILESS_MAIN(FrontendTest)
#include "qt_frontend_test.moc"
