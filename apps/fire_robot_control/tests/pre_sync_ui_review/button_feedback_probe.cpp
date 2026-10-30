// Standalone Qt rendering/input QA. No ROS, socket, runner or device access.
#include "button_style.hpp"
#include <QApplication>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QTest>
#include <QVBoxLayout>
#include <cstdio>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  QWidget panel;
  panel.setStyleSheet("QWidget { background: #ffffff; color: #252525; }" + robotButtonStyle());
  auto * layout = new QVBoxLayout(&panel);
  auto * title = new QLabel(QString::fromUtf8("PHẢN HỒI NÚT · FIRE ROBOT / PTIT"));
  title->setStyleSheet("background:#b62025; color:white; padding:12px; font-weight:bold;");
  layout->addWidget(title);
  auto * examples = new QHBoxLayout;
  const QStringList labels{QString::fromUtf8("Bình thường"), QString::fromUtf8("Đang giữ chuột"),
    QString::fromUtf8("Vừa bấm"), QString::fromUtf8("Đang xử lý / khóa")};
  for (int i = 0; i < labels.size(); ++i) {
    auto * box = new QVBoxLayout;
    box->addWidget(new QLabel(labels[i]));
    auto * button = new QPushButton(i == 3 ? QString::fromUtf8("Đang lưu map…") : QString::fromUtf8("Quét map"));
    button->setFocusPolicy(Qt::NoFocus);
    button->setDown(i == 1);
    button->setProperty("clickFeedback", i == 2);
    button->setEnabled(i != 3);
    box->addWidget(button); examples->addLayout(box);
  }
  layout->addLayout(examples);
  auto * input = new QPushButton(QString::fromUtf8("Nút kiểm tra chuột / bàn phím"));
  installButtonFeedback(input);
  layout->addWidget(input);
  panel.resize(800, 210); panel.show(); QTest::qWait(30);
  if (argc == 2) {panel.grab().save(QString::fromLocal8Bit(argv[1]));}
  int clicks = 0, beats = 0;
  QObject::connect(input, &QPushButton::clicked, &panel, [&]() {++clicks;});
  QTimer heartbeat;
  QObject::connect(&heartbeat, &QTimer::timeout, &panel, [&]() {++beats;});
  heartbeat.start(20);
  input->setFocus(); QTest::mouseMove(input, input->rect().center()); app.processEvents();
  const auto normal = input->grab().toImage();
  QTest::mousePress(input, Qt::LeftButton);
  const auto pressed = input->grab().toImage();
  QJsonObject result{{"held_down", input->isDown()}, {"pressed_changes_pixels", normal != pressed},
    {"no_action_before_release", clicks == 0}, {"stable_size", normal.size() == pressed.size()}};
  QTest::mouseRelease(input, Qt::LeftButton); app.processEvents();
  result["one_action_on_release"] = clicks == 1;
  result["fast_click_visible"] = input->property("clickFeedback").toBool();
  QTest::qWait(240);
  result["feedback_cleared"] = !input->property("clickFeedback").toBool();
  result["event_loop_responsive"] = beats >= 5;
  input->setEnabled(false);
  QTest::mouseClick(input, Qt::LeftButton); QTest::keyClick(input, Qt::Key_Space);
  result["disabled_ignores_input"] = clicks == 1;
  input->setEnabled(true); input->setFocus(); QTest::keyClick(input, Qt::Key_Space);
  result["keyboard_one_action"] = clicks == 2 && input->property("clickFeedback").toBool();
  std::printf("BUTTON_FEEDBACK_QA %s\n", QJsonDocument(result).toJson(QJsonDocument::Compact).constData());
  return 0;
}
