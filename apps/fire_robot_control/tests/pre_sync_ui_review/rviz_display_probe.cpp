// Real native app layout and RViz plugins; all incoming ROS data is synthetic.
#define main production_entry_unused_in_visual_qa
#include "../../native_rviz/main.cpp"
#undef main
#include <QLocalServer>
#include <QTemporaryDir>
#include <QTest>
#include <QJsonParseError>
#include <nav_msgs/msg/path.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/properties/status_list.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <OgreSceneNode.h>
#include <OgreMovableObject.h>
#include <map>

using Positions=std::map<std::string,Ogre::Vector3>;
void positions(Ogre::SceneNode * node, Positions & result) {
  for(unsigned i=0;i<node->numAttachedObjects();++i) {
    auto * object=node->getAttachedObject(i);
    if(object->isVisible())result[object->getName()]=node->_getDerivedPosition();
  }
  for(auto * n:node->getChildren())if(auto * child=dynamic_cast<Ogre::SceneNode*>(n))positions(child,result);
}
bool healthy(rviz_common::Display * display) {
  for(int i=0;i<display->numChildren();++i) {
    auto * status=dynamic_cast<rviz_common::properties::StatusList*>(display->childAt(i));
    if(!status)continue;
    std::fprintf(stderr,"QA_STATUS display=%s level=%d items=%d\n",display->getName().toUtf8().constData(),static_cast<int>(status->getLevel()),status->numChildren());
    return status->numChildren()>0&&status->getLevel()==rviz_common::properties::StatusProperty::Ok;
  }
  return false;
}
int main(int argc,char **argv) {
  if(argc!=4 || qgetenv("ROS_DOMAIN_ID")!="211" || qgetenv("ROS_LOCALHOST_ONLY")!="1")return 2;
  const QString config=QString::fromLocal8Bit(argv[1]),fixture=QString::fromLocal8Bit(argv[2]),capture=QString::fromLocal8Bit(argv[3]);
  QFile f(fixture);if(!f.open(QIODevice::ReadOnly))return 3;
  auto data=QJsonDocument::fromJson(f.readAll()).object();
  int one=1;rclcpp::init(one,argv);QApplication app(one,argv);
  auto source=rclcpp::Node::make_shared("robot_state_publisher");
  auto planner=rclcpp::Node::make_shared("planner_server");
  auto description=source->create_publisher<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).reliable().transient_local());
  auto staticTf=source->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",rclcpp::QoS(1).reliable().transient_local());
  auto tf=source->create_publisher<tf2_msgs::msg::TFMessage>("/tf",10);
  auto map=source->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",rclcpp::QoS(1).reliable().transient_local());
  auto path=planner->create_publisher<nav_msgs::msg::Path>("/plan",1);
  QJsonObject results;
  {
    auto abstraction=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("fire_robot_visual_qa");
    QMainWindow frame;std::unique_ptr<Observer> observer;
    QTemporaryDir temp;QLocalServer server;
    if(!server.listen(temp.path()+"/control.sock"))return 4;
    FrontendBootstrap boot{"visual-qa","generation","nonce",QString(64,'a'),temp.path()+"/control.sock"};
    AppClient client(boot);QTest::qWait(30);
    std::unique_ptr<QLocalSocket> peer(server.nextPendingConnection());if(!peer)return 5;
    auto * ui=configureUi(frame,nullptr,observer,config,false,&client);
    auto * rviz=new rviz_common::VisualizationFrame(abstraction);
    rviz->setApp(&app);rviz->setSplashPath("");rviz->initialize(abstraction,config);rviz->menuBar()->hide();
    frame.setCentralWidget(rviz);ui->setView(rviz);frame.resize(1400,850);frame.show();
    auto * root=rviz->getManager()->getRootDisplayGroup();
    rviz_common::Display * robot=nullptr,*plan=nullptr;
    for(int i=0;i<root->numDisplays();++i) {
      auto * d=root->getDisplayAt(i);if(d->getName()=="Robot")robot=d;if(d->getName()=="Nav2 Plan")plan=d;
    }
    if(!robot||!plan)return 6;
    QTest::qWait(200);
    results["idle_overlays_disabled"]=!robot->isEnabled()&&!plan->isEnabled();
    const auto readerCount=[&](const std::string & topic) {
      int n=0;for(const auto & info:source->get_subscriptions_info_by_topic(topic))n+=info.node_name()=="fire_robot_visual_qa";return n;
    };
    results["idle_overlay_readers_zero"]=readerCount("/plan")==0&&readerCount("/robot_description")==0;
    QJsonObject row{{"event","STATUS"},{"phase","IDLE"},{"stage","IDLE"},{"maps",QJsonArray()},
      {"session",QJsonValue()},{"selected","/fake/map.yaml"}};
    client.message(row);frame.findChild<QPushButton*>("scan_button")->click();
    client.message(QJsonObject{{"event","NEW_SESSION"},{"session","map-qa"},{"kind","mapping"}});
    row["phase"]="STARTING";row["stage"]="WAIT_SLAM";row["session"]="map-qa";client.message(row);
    results["mapping_robot_on_path_off"]=robot->isEnabled()&&!plan->isEnabled();
    frame.findChild<QPushButton*>("stop_button")->click();
    row["phase"]="IDLE";row["stage"]="IDLE";row["session"]=QJsonValue();client.message(row);
    frame.findChild<QPushButton*>("start_button")->click();
    client.message(QJsonObject{{"event","NEW_SESSION"},{"session","fire-qa"},{"kind","fire"}});
    row["phase"]="STARTING";row["stage"]="POSE";row["session"]="fire-qa";client.message(row);
    results["fire_overlays_enabled"]=robot->isEnabled()&&plan->isEnabled();
    std_msgs::msg::String urdf;urdf.data=data.value("urdf").toString().toStdString();description->publish(urdf);
    tf2_msgs::msg::TFMessage statics;
    for(const auto & v:data.value("joints").toArray()) {
      auto obj=v.toObject();geometry_msgs::msg::TransformStamped t;t.header.stamp=source->now();
      t.header.frame_id=obj["parent"].toString().toStdString();t.child_frame_id=obj["child"].toString().toStdString();
      auto xyz=obj["xyz"].toArray(),q=obj["quaternion"].toArray();
      t.transform.translation.x=xyz[0].toDouble();t.transform.translation.y=xyz[1].toDouble();t.transform.translation.z=xyz[2].toDouble();
      t.transform.rotation.x=q[0].toDouble();t.transform.rotation.y=q[1].toDouble();t.transform.rotation.z=q[2].toDouble();t.transform.rotation.w=q[3].toDouble();statics.transforms.push_back(t);
    }
    staticTf->publish(statics);
    nav_msgs::msg::OccupancyGrid grid;grid.header.frame_id="map";grid.info.resolution=.05;grid.info.width=100;grid.info.height=100;
    grid.info.origin.position.x=-2.;grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1;grid.data.resize(10000,0);
    for(int i=0;i<100;++i) {grid.data[i]=100;grid.data[9900+i]=100;grid.data[100*i]=100;grid.data[100*i+99]=100;}
    grid.header.stamp=source->now();map->publish(grid);
    auto publish=[&](double x) {
      tf2_msgs::msg::TFMessage message;geometry_msgs::msg::TransformStamped a,b;
      a.header.stamp=b.header.stamp=source->now();a.header.frame_id="map";a.child_frame_id="odom";a.transform.rotation.w=1;
      b.header.frame_id="odom";b.child_frame_id="base_link";b.transform.translation.x=x;b.transform.rotation.w=1;
      message.transforms={a,b};tf->publish(message);
      nav_msgs::msg::Path p;p.header.frame_id="map";p.header.stamp=source->now();
      for(int i=0;i<30;++i) {geometry_msgs::msg::PoseStamped pose;pose.header=p.header;pose.pose.orientation.w=1;
        pose.pose.position.x=.05*i;pose.pose.position.y=i<15?0.:.05*(i-15);p.poses.push_back(pose);}
      path->publish(p);
    };
    const auto pump=[&](double x,int count) {for(int i=0;i<count;++i) {publish(x);QTest::qWait(60);}};
    pump(0.,35);Positions first;positions(robot->getSceneNode(),first);
    pump(.55,15);Positions second;positions(robot->getSceneNode(),second);
    results["robot_urdf_and_tf_ok"]=healthy(robot)&&first.size()>=5;
    bool moved=false;for(const auto & item:first)if(second.count(item.first))moved|=std::abs(second[item.first].x-item.second.x)>.4;
    results["robot_tracks_tf"]=moved;
    results["path_messages_ok"]=healthy(plan)&&readerCount("/plan")==1;
    results["no_drive_or_pump_publishers"]=source->get_publishers_info_by_topic("/cmd_vel_raw").empty()
      &&source->get_publishers_info_by_topic("/cmd_vel").empty()&&source->get_publishers_info_by_topic("/pump_cmd").empty();
    frame.screen()->grabWindow(frame.winId()).save(capture);
    frame.grab().save(capture+".widgets.png");
    rviz->getManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(capture.toStdString()+".map.png");
    frame.findChild<QPushButton*>("stop_button")->click();
    QTest::qWait(500);
    results["stop_disables_and_clears_overlays"]=!robot->isEnabled()&&!plan->isEnabled();
    results["stop_overlay_readers_zero"]=readerCount("/plan")==0&&readerCount("/robot_description")==0;
    frame.screen()->grabWindow(frame.winId()).save(capture+".stopped.png");
    rviz->getManager()->getRenderPanel()->getRenderWindow()->captureScreenShot(capture.toStdString()+".stopped.map.png");
    row["phase"]="IDLE";row["stage"]="IDLE";row["session"]=QJsonValue();row["close_ready"]=true;client.message(row);QTest::qWait(100);
    ui->setView(nullptr);
  }
  source.reset();planner.reset();rclcpp::shutdown();
  std::puts(("QA_VISUAL "+QJsonDocument(results).toJson(QJsonDocument::Compact)).constData());return 0;
}
