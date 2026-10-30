#pragma once
#include <QJsonObject>
#include <QString>

inline QString navigationRecoveryLabel(const QJsonObject & recovery)
{
  const auto state = recovery.value("state").toString();
  if (state == "WAIT_COMMAND") {
    return QString::fromUtf8("Tạm dừng · chờ dữ liệu và lệnh Nav2 mới để tiếp tục cùng Goal");
  }
  if (state == "WAIT_STOP" || state == "VERIFYING" || state == "REACQUIRING") {
    return QString::fromUtf8("Phục hồi Goal · đang xác minh dừng và dữ liệu mới");
  }
  if (state == "BLOCKED") {
    return QString::fromUtf8("Goal đang giữ dừng · %1 · X để hủy tác vụ, Q dừng phiên")
      .arg(recovery.value("reason").toString());
  }
  return {};
}
