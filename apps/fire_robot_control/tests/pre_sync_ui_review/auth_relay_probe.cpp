// Real SessionUi/AppClient modal with real supervisor; fixture-only password.
#include "session_ui.hpp"
#include <QApplication>
#include <QComboBox>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>
#include <unistd.h>
int main(int argc,char **argv) {
  if(argc!=4||qgetenv("ROS_DOMAIN_ID")!="211"||qgetenv("ROS_LOCALHOST_ONLY")!="1")return 2;
  QApplication app(argc,argv);bool valid=false;int fd=QByteArray(argv[3]).toInt(&valid);
  if(!valid)return 2;auto boot=readFrontendBootstrap(fd);::close(fd);if(!boot)return 2;
  AppClient client(*boot);QMainWindow frame;auto *central=new QWidget(&frame);frame.setCentralWidget(central);
  auto *layout=new QVBoxLayout(central);QComboBox maps;QPushButton scan,save,start,cancel,stop;QLabel status;
  for(auto *w:QList<QWidget*>{&maps,&scan,&save,&start,&cancel,&stop,&status})layout->addWidget(w);
  SessionUi ui(frame,client,&maps,&scan,&save,&start,&cancel,&stop,&status,[](bool){});
  frame.resize(800,500);frame.show();bool started=false,finished=false,dialog=false,masked=false,authDone=false;
  const bool reject=qgetenv("FIRE_AUTH_QA_MODE")=="cancel";
  const bool timeout=qgetenv("FIRE_AUTH_QA_MODE")=="timeout";QTimer modal;
  modal.setInterval(20);QObject::connect(&modal,&QTimer::timeout,&app,[&](){
    auto *d=qobject_cast<QInputDialog*>(QApplication::activeModalWidget());if(!d)return;
    dialog=true;auto *edit=d->findChild<QLineEdit*>();masked=edit&&edit->echoMode()==QLineEdit::Password;
    modal.stop();if(timeout)return;
    if(reject)d->reject();else {d->setTextValue("fixture-only");d->accept();}
  });modal.start();
  QObject::connect(&client,&AppClient::message,&app,[&](const QJsonObject &row){
    const auto event=row["event"].toString();
    if(event=="STATUS"&&row["phase"]=="IDLE"&&!started){started=true;scan.click();}
    if(event=="AUTH_DONE")authDone=true;
    if(event=="FINISHED"){finished=true;client.stop(true);}
    if(event=="STATUS"&&row["close_ready"].toBool()&&finished){
      if(timeout&&QApplication::activeModalWidget()){app.exit(5);return;}
      std::printf("AUTH_QA dialog=%d masked=%d done=%d finished=%d cancel=%d\n",dialog,masked,authDone,finished,reject);
      app.quit();
    }
  });QTimer::singleShot(8000,&app,[&](){app.exit(4);});return app.exec();
}
