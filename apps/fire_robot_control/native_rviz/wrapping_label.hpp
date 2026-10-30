#pragma once
#include <QFontMetrics>
#include <QLabel>
#include <QPainter>
#include <algorithm>

// Keep the original text for selection/tests; long tokens never set the
// window's minimum width. The same wrapping flags drive measure and paint.
class WrappingLabel : public QLabel
{
public:
  explicit WrappingLabel(const QString & text, QWidget * parent = nullptr)
  : QLabel(text, parent)
  {
    setWordWrap(true);
    auto policy = QSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);
  }
  QSize minimumSizeHint() const override {return {0, fontMetrics().height()};}
  QSize sizeHint() const override {return {360, heightForWidth(360)};}
  int heightForWidth(int width) const override
  {
    return fontMetrics().boundingRect(QRect(0, 0, std::max(1, width), 100000),
      Qt::TextWordWrap | Qt::TextWrapAnywhere, text()).height();
  }
protected:
  void paintEvent(QPaintEvent *) override
  {
    QPainter painter(this);
    painter.setFont(font());
    painter.setPen(palette().color(QPalette::WindowText));
    painter.drawText(rect(), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap | Qt::TextWrapAnywhere, text());
  }
};
