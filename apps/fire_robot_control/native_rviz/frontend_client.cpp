#include "frontend_client.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>
#include <time.h>

#include <stdexcept>

namespace
{
constexpr qint64 MAX_QUEUED_BYTES = 4096;
constexpr qint64 MAX_SEQUENCE = 9007199254740992LL;
}

FrontendClient::FrontendClient(const QString & session, const QString & generation,
  const QString & nonce, QObject * parent)
: QObject(parent), session_(session), generation_(generation), nonce_(nonce)
{
  if (session.isEmpty() || generation.isEmpty() || nonce.isEmpty() ||
    session.size() > 128 || generation.size() > 128 || nonce.size() > 128)
  {
    throw std::invalid_argument("invalid frontend identity");
  }
  heartbeat_.setInterval(100);
  heartbeat_.setTimerType(Qt::PreciseTimer);
  connect(&heartbeat_, &QTimer::timeout, this, [this]() {sendEvent("HEARTBEAT");});
  connect(&socket_, &QLocalSocket::connected, this, [this]() {
    if (sendEvent("HEARTBEAT")) {
      if (stopRequested_) {sendEvent("Q");} else {heartbeat_.start();}
    }
  });
  connect(&socket_, &QLocalSocket::disconnected, this, [this]() {heartbeat_.stop();});
}

bool FrontendClient::onOwnerThread() const
{
  return QThread::currentThread() == thread();
}

void FrontendClient::connectTo(const QString & path)
{
  if (!onOwnerThread() || path.isEmpty() || socket_.state() != QLocalSocket::UnconnectedState ||
    sequence_ != 0 || stopRequested_)
  {
    throw std::logic_error("invalid frontend connection state");
  }
  socket_.connectToServer(path, QIODevice::ReadWrite);
}

bool FrontendClient::connected() const
{
  return onOwnerThread() && socket_.state() == QLocalSocket::ConnectedState;
}

bool FrontendClient::sendCancel()
{
  return onOwnerThread() && !stopRequested_ && sendEvent("X");
}

bool FrontendClient::sendStop()
{
  if (!onOwnerThread() || stopRequested_) {return false;}
  // Once Q has been attempted, never send a later X on this generation.
  stopRequested_ = true;
  return sendEvent("Q");
}

bool FrontendClient::sendEvent(const char * event)
{
  if (!connected()) {return false;}
  struct timespec now{};
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0 || sequence_ >= MAX_SEQUENCE) {
    socket_.abort();
    return false;
  }
  const double sent = static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_nsec) / 1e9;
  const QJsonObject message{{"version", 1}, {"session", session_},
    {"generation", generation_}, {"nonce", nonce_}, {"seq", ++sequence_},
    {"sent_mono", sent}, {"event", QString::fromLatin1(event)}};
  QByteArray bytes = QJsonDocument(message).toJson(QJsonDocument::Compact);
  bytes.append('\n');
  if (bytes.size() > 2049 || socket_.bytesToWrite() + bytes.size() > MAX_QUEUED_BYTES ||
    socket_.write(bytes) != bytes.size())
  {
    // A disconnected client makes the operator's lease request Q; do not
    // silently queue unbounded data or claim that any command was accepted.
    socket_.abort();
    return false;
  }
  socket_.flush();
  return true;
}
