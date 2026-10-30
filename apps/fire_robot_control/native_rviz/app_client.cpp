#include "app_client.hpp"
#include <QJsonDocument>
#include <QThread>
#include <time.h>

AppClient::AppClient(const FrontendBootstrap & bootstrap, QObject * parent)
: QObject(parent), bootstrap_(bootstrap)
{
  heartbeat_.setInterval(100);
  heartbeat_.setTimerType(Qt::PreciseTimer);
  connect(&heartbeat_, &QTimer::timeout, this, [this]() {send("HEARTBEAT");});
  connect(&socket_, &QLocalSocket::connected, this, [this]() {
    if (send("HEARTBEAT")) {heartbeat_.start();}
  });
  connect(&socket_, &QLocalSocket::readyRead, this, &AppClient::read);
  connect(&socket_, &QLocalSocket::disconnected, this, [this]() {
    heartbeat_.stop();
    if (operator_) {operator_->sendStop();}
    Q_EMIT lost();
  });
  socket_.connectToServer(bootstrap.socketPath);
}

bool AppClient::send(const QString & event, const QJsonValue & argument)
{
  if (QThread::currentThread() != thread() || socket_.state() != QLocalSocket::ConnectedState ||
    sequence_ >= 9007199254740992LL) {return false;}
  struct timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {socket_.abort(); return false;}
  const double sent = static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_nsec) / 1e9;
  const QJsonObject value{{"version", 1}, {"session", bootstrap_.session},
    {"generation", bootstrap_.generation}, {"nonce", bootstrap_.nonce}, {"seq", ++sequence_},
    {"sent_mono", sent}, {"event", event}, {"argument", argument}};
  auto bytes = QJsonDocument(value).toJson(QJsonDocument::Compact) + '\n';
  if (bytes.size() > 4096 || socket_.bytesToWrite() + bytes.size() > 16384 ||
    socket_.write(bytes) != bytes.size()) {socket_.abort(); return false;}
  socket_.flush();
  return true;
}

void AppClient::read()
{
  input_ += socket_.readAll();
  if (input_.size() > 1024 * 1024) {socket_.abort(); return;}
  while (input_.contains('\n')) {
    const auto split = input_.indexOf('\n');
    const auto bytes = input_.left(split);
    input_.remove(0, split + 1);
    const auto doc = QJsonDocument::fromJson(bytes);
    if (!doc.isObject()) {socket_.abort(); return;}
    const auto value = doc.object();
    const auto event = value.value("event").toString();
    if (event == "NEW_SESSION" || event == "FINISHED") {operator_.reset();}
    if (event == "OPERATOR" && !operator_) {
      operator_ = std::make_unique<FrontendClient>(value.value("session").toString(),
        value.value("generation").toString(), value.value("nonce").toString());
      operator_->connectTo(value.value("socket_path").toString());
      if (closing_) {operator_->sendStop();}
    }
    Q_EMIT message(value);
  }
}

bool AppClient::cancel()
{
  return !closing_ && operator_ && operator_->sendCancel();
}

void AppClient::stop(bool closeWindow)
{
  closing_ |= closeWindow;
  if (operator_) {operator_->sendStop();}
  send(closeWindow ? "CLOSE" : "STOP");
}

bool AppClient::operatorConnected() const
{
  return operator_ && operator_->connected();
}
