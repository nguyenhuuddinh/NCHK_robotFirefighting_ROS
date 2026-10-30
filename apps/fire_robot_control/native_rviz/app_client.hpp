#pragma once
#include "frontend_bootstrap.hpp"
#include "frontend_client.hpp"
#include <QJsonObject>
#include <QLocalSocket>
#include <QObject>
#include <QTimer>
#include <memory>

class AppClient : public QObject
{
  Q_OBJECT
public:
  explicit AppClient(const FrontendBootstrap & bootstrap, QObject * parent = nullptr);
  bool send(const QString & event, const QJsonValue & argument = QJsonValue());
  bool cancel();
  void stop(bool closeWindow = false);
  bool operatorConnected() const;
  const QString & packet() const {return bootstrap_.packet;}
Q_SIGNALS:
  void message(const QJsonObject & value);
  void lost();
private:
  void read();
  FrontendBootstrap bootstrap_;
  QLocalSocket socket_;
  QTimer heartbeat_;
  QByteArray input_;
  qint64 sequence_ = 0;
  bool closing_ = false;
  std::unique_ptr<FrontendClient> operator_;
};
