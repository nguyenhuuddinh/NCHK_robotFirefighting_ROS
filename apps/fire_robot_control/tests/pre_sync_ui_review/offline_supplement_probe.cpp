#include "session_ui.hpp"
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QFocusEvent>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QLocalServer>
#include <QLocalSocket>
#include <QMainWindow>
#include <QPushButton>
#include <QStatusBar>
#include <QTemporaryDir>
#include <QTest>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <memory>

int main(int argc, char **argv) {
  if(qgetenv("ROS_DOMAIN_ID")!="211" || qgetenv("ROS_LOCALHOST_ONLY")!="1") return 2;
  QApplication app(argc,argv);
  QTemporaryDir temp;
  QLocalServer server;
  if(!server.listen(temp.path()+"/control.sock")) return 3;
  FrontendBootstrap boot{"local-qa","generation","nonce",QString(64,'a'),temp.path()+"/control.sock"};
  AppClient client(boot);
  QTest::qWait(30);
  std::unique_ptr<QLocalSocket> peer(server.nextPendingConnection());
  if(!peer) return 4;
  QMainWindow frame;
  auto * central=new QWidget(&frame);frame.setCentralWidget(central);
  auto * layout=new QVBoxLayout(central);
  QComboBox maps; QPushButton scan,save,start,cancel,stop; QLabel status;
  for(auto * widget: QList<QWidget*>{&maps,&scan,&save,&start,&cancel,&stop,&status})layout->addWidget(widget);
  SessionUi ui(frame,client,&maps,&scan,&save,&start,&cancel,&stop,&status,[](bool){});
  frame.resize(900,700);frame.show();QTest::qWait(40);
  QJsonObject row{{"event","STATUS"},{"phase","IDLE"},{"stage","IDLE"},{"maps",QJsonArray()},
                  {"session",QJsonValue()},{"selected","/fake/map.yaml"}};
  client.message(row);scan.click();
  client.message(QJsonObject{{"event","NEW_SESSION"},{"session","mapping-1"},{"kind","mapping"}});
  row["phase"]="MAPPING";row["stage"]="MAPPING";row["session"]="mapping-1";client.message(row);
  QTest::qWait(40);peer->readAll();
  QJsonObject results;
  QTimer::singleShot(30,&app,[&]() {
    auto * dialog=qobject_cast<QInputDialog *>(QApplication::activeModalWidget());
    if(!dialog) {results["save_dialog_exists"]=false;return;}
    results["save_dialog_exists"]=true;
    auto * text=dialog->findChild<QLineEdit*>();
    if(text) {text->setFocus();QTest::keyClicks(text,"map_X_Q_2026");}
    QTest::qWait(40);
    const auto before=peer->readAll();
    results["typing_xq_does_not_stop"]=!before.contains("\"event\":\"STOP\"")&&!before.contains("\"event\":\"CLOSE\"");
    dialog->accept();
  });
  save.click();QTest::qWait(40);
  auto bytes=peer->readAll();
  int saves=0;bool exact=false;
  for(const auto &line:bytes.split('\n')) {
    const auto message=QJsonDocument::fromJson(line).object();
    if(message.value("event")=="SAVE") {saves++;exact=message.value("argument")=="map_X_Q_2026";}
  }
  results["save_name_xq_sent_once"]=saves==1&&exact;
  client.message(row);QTest::qWait(20);peer->readAll();
  QDialog * keyboard=nullptr;
  for(auto * dialog:frame.findChildren<QDialog*>())if(dialog->windowTitle().contains(QString::fromUtf8("Bàn phím")))keyboard=dialog;
  results["keyboard_exists"]=keyboard!=nullptr;
  if(keyboard) {
    QFocusEvent focus(QEvent::FocusOut,Qt::OtherFocusReason);
    QApplication::sendEvent(keyboard,&focus);QTest::qWait(30);
    bytes=peer->readAll();bool found=false;
    for(const auto &line:bytes.split('\n')) {
      const auto message=QJsonDocument::fromJson(line).object();
      found|=message.value("event")=="KEY"&&message.value("argument")=="k";
    }
    results["keyboard_focus_out_sends_zero_key"]=found;
  }
  // A bound asynchronous authentication cancel must request STOP.
  QTimer::singleShot(30,&app,[&]() {
    if(auto * dialog=qobject_cast<QInputDialog *>(QApplication::activeModalWidget()))dialog->reject();
  });
  client.message(QJsonObject{{"event","AUTH_REQUIRED"},{"session","mapping-1"},{"label","local fake authentication"}});
  QTest::qWait(40);bytes=peer->readAll();
  results["cancel_auth_requests_stop"]=bytes.contains("\"event\":\"STOP\"");
  std::puts(("QA_SUPPLEMENT "+QJsonDocument(results).toJson(QJsonDocument::Compact)).constData());
  // Let the real close path run; no fake robot was ever started.
  row["phase"]="IDLE";row["stage"]="IDLE";row["session"]=QJsonValue();row["close_ready"]=true;client.message(row);
  QTest::qWait(20);
  return 0;
}
