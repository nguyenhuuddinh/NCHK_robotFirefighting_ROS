// Compile the existing layout unchanged; only this QA entrypoint is replaced.
#define main production_entry_unused_in_qa
#include "../../native_rviz/main.cpp"
#undef main
#include <QInputDialog>
#include <QLocalServer>
#include <QTemporaryDir>
#include <QTest>
#include <QScreen>

int main(int argc,char **argv) {
  if(argc!=5 || qgetenv("ROS_DOMAIN_ID")!="211" || qgetenv("ROS_LOCALHOST_ONLY")!="1")return 2;
  const int width=std::atoi(argv[1]),height=std::atoi(argv[2]);
  const bool longError=std::string(argv[3])=="long";
  QApplication app(argc,argv);
  QTemporaryDir temp;QLocalServer server;
  if(!server.listen(temp.path()+"/control.sock"))return 3;
  FrontendBootstrap boot{"local-qa","generation","nonce",QString(64,'a'),temp.path()+"/control.sock"};
  AppClient client(boot);QTest::qWait(30);
  std::unique_ptr<QLocalSocket> peer(server.nextPendingConnection());if(!peer)return 4;
  QMainWindow frame;std::unique_ptr<Observer> observer;
  auto * placeholder=new QLabel(QString::fromUtf8("Bản đồ sẽ hiện sau khi Pi sẵn sàng"));
  placeholder->setAlignment(Qt::AlignCenter);frame.setCentralWidget(placeholder);
  configureUi(frame,nullptr,observer,"/unused",false,&client);
  const double dpr=frame.devicePixelRatioF();
  const QSize logical(qRound(width/dpr),qRound(height/dpr));
  frame.resize(logical);frame.show();QTest::qWait(80);
  QJsonObject row{{"event","STATUS"},{"phase","IDLE"},{"stage","IDLE"},{"maps",QJsonArray()},
                  {"session",QJsonValue()},{"selected","/fake/map.yaml"}};
  client.message(row);
  frame.findChild<QPushButton*>("start_button")->click();
  client.message(QJsonObject{{"event","NEW_SESSION"},{"session","fire-1"},{"kind","fire"}});
  row["phase"]="STARTING";row["stage"]="SEAL_PI";row["session"]="fire-1";
  row["progress"]=QJsonObject{{"remaining_s",12.5},{"blockers",QJsonArray{"io","camera","odom_tf"}}};
  if(longError)row["error"]=QString("PI_SEAL_HEALTH_TIMEOUT ")+QString(360,'W');
  client.message(row);QTest::qWait(80);
  auto * stop=frame.findChild<QPushButton*>("stop_button");auto * camera=frame.findChild<QWidget*>("camera_widget");
  const QRect stopRect(stop->mapTo(&frame,QPoint()),stop->size());
  const bool fits=frame.width()<=logical.width()+2 && frame.height()<=logical.height()+2;
  const bool stopVisible=frame.rect().contains(stopRect)&&stop->isVisible()&&stop->isEnabled();
  auto * warning=frame.findChild<QLabel*>("warning");
  const bool progress=warning->text().contains("12.5")&&warning->text().contains("odom_tf");
  QJsonObject result{{"physical_width",width},{"physical_height",height},{"dpr",dpr},{"logical_width",logical.width()},
    {"logical_height",logical.height()},{"actual_width",frame.width()},{"actual_height",frame.height()},
    {"fits_requested_screen",fits},{"stop_visible_enabled",stopVisible},{"progress_visible",progress},
    {"camera_height",camera->height()},{"camera_width",camera->width()},{"long_error",longError}};
  frame.grab().save(QString::fromLocal8Bit(argv[4]));
  std::puts(("QA_LAYOUT "+QJsonDocument(result).toJson(QJsonDocument::Compact)).constData());
  row["phase"]="IDLE";row["stage"]="IDLE";row["session"]=QJsonValue();row["close_ready"]=true;client.message(row);
  QTest::qWait(20);return 0;
}
