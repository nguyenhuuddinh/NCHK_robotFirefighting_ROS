#pragma once

#include <QLocalSocket>
#include <QObject>
#include <QString>
#include <QTimer>

// Transport only. It does not start an operator or prove that the robot stopped.
// Construct and call on the Qt UI thread; the timer intentionally lives there.
class FrontendClient : public QObject
{
  Q_OBJECT
public:
  FrontendClient(const QString & session, const QString & generation,
    const QString & nonce, QObject * parent = nullptr);
  void connectTo(const QString & path);
  bool sendCancel();
  bool sendStop();
  bool connected() const;

private:
  bool sendEvent(const char * event);
  bool onOwnerThread() const;
  QLocalSocket socket_;
  QTimer heartbeat_;
  QString session_;
  QString generation_;
  QString nonce_;
  qint64 sequence_ = 0;
  bool stopRequested_ = false;
};
