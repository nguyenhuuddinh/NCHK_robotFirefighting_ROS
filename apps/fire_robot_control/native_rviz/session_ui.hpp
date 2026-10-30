#pragma once
#include "app_client.hpp"
#include <QObject>
#include <QJsonArray>
#include <QJsonObject>
#include <QPointer>
#include <functional>
class QComboBox;
class QPushButton;
class QLabel;
class QDialog;
class QMainWindow;
class QInputDialog;
namespace rviz_common {class VisualizationFrame;}

class SessionUi : public QObject
{
public:
  SessionUi(QMainWindow & frame, AppClient & client,
    QComboBox * maps, QPushButton * scan, QPushButton * save, QPushButton * start,
    QPushButton * cancel, QPushButton * stop, QLabel * status,
    std::function<void(bool)> newSession);
  void updateGoalPermission(bool allowed);
  void setView(rviz_common::VisualizationFrame * view);
  bool viewAllowed() const;
  void prepareRestart();
  bool restarting() const {return restarting_;}
  bool closing() const {return closing_;}
  bool mappingView() const {return mappingSession_;}
  QString sessionId() const {return session_;}
protected:
  bool eventFilter(QObject * target, QEvent * event) override;
private:
  void update(const QJsonObject & value);
  void lockCommands(bool disableScan = true);
  void sentFeedback(const QString & text);
  bool poseEnabled() const;
  bool goalEnabled() const;
  bool ownsSession() const;
  void tools();
  void closeAuth();
  QMainWindow & frame_;
  rviz_common::VisualizationFrame * view_ = nullptr;
  QPushButton * verify_ = nullptr;
  AppClient & client_;
  QComboBox * maps_;
  QPushButton * scan_;
  QPushButton * save_;
  QPushButton * start_;
  QPushButton * cancel_;
  QPushButton * stop_;
  QLabel * status_;
  QDialog * keyboard_ = nullptr;
  QPointer<QInputDialog> auth_;
  std::function<void(bool)> newSession_;
  QJsonObject state_;
  QJsonArray mapCatalog_;
  bool mapCatalogReady_ = false;
  QString session_;
  bool closeReady_ = false;
  bool restarting_ = false;
  bool closing_ = false;
  bool stopping_ = true;
  bool disconnected_ = false;
  bool newSessionRequested_ = false;
  bool goalAllowed_ = false;
  bool cancelCooling_ = false;
  bool mappingSession_ = false;
};
