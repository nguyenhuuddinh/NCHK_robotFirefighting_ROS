#pragma once
#include <QJsonObject>
#include <cmath>

inline bool appGoalAllowed(const QJsonObject & report, double age)
{
  return std::isfinite(age) && age >= 0 && age <= .5 &&
    report.value("schema_version").toInt() == 1 &&
    report.value("scope").toString() == "R5_SESSION_INTENT_ONLY" &&
    report.value("armed").isBool() && report.value("armed").toBool() &&
    report.value("closed").isBool() && !report.value("closed").toBool() &&
    report.value("automatic").toObject().value("enabled").toBool() &&
    report.value("app_goal_admission").toObject().value("allowed").toBool() &&
    report.value("fault").isNull() && !report.value("input_hold").toBool() &&
    !report.value("identity_hold").toBool() && !report.value("graph_hold").toBool() &&
    !report.value("runtime_rebind").toObject().value("held").toBool() &&
    report.value("cycle").isNull() &&
    (report.value("phase").toString() == "ARMED_IDLE" ||
    report.value("phase").toString() == "NAVIGATION");
}
