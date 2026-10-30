#pragma once
#include <QPushButton>
#include <QStyle>
#include <QTimer>
#include <QVariant>

// The custom border replaces the platform button painting, so pressed,
// hover, keyboard focus and disabled states must all be drawn explicitly.
inline QString robotButtonStyle()
{
  return QStringLiteral(
    "QPushButton { background: #f7f8fa; color: #252525; padding: 6px 10px;"
    " border: 1px solid #a5abb3; border-bottom: 3px solid #858c96; border-radius: 4px; }"
    "QPushButton:hover:enabled { background: #fff0f1; border-color: #b62025; }"
    "QPushButton:focus:enabled { border-color: #b62025; }"
    "QPushButton[clickFeedback=\"true\"]:enabled { background: #f3cdd0; border-color: #b62025; }"
    "QPushButton:pressed:enabled { background: #d6d9de; color: #81171b;"
    " border: 1px solid #858c96; border-top: 3px solid #69717c; padding-top: 8px; padding-bottom: 4px; }"
    "QPushButton:disabled { color: #777777; background: #eeeeee;"
    " border: 1px solid #c9cdd2; border-bottom: 3px solid #c9cdd2; }");
}

inline void installButtonFeedback(QPushButton * button)
{
  button->setCursor(Qt::PointingHandCursor);
  QObject::connect(button, &QPushButton::clicked, button, [button]() {
    button->setProperty("clickFeedback", true);
    button->style()->unpolish(button); button->style()->polish(button); button->update();
    // A brief tint remains visible even for a fast click or keyboard activation.
    QTimer::singleShot(180, button, [button]() {
      button->setProperty("clickFeedback", false);
      button->style()->unpolish(button); button->style()->polish(button); button->update();
    });
  });
}
