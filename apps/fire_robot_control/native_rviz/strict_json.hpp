#pragma once
#include <QByteArray>
#include <QJsonObject>
#include <optional>

// UI-only report validation: reject ambiguous duplicate keys and invalid UTF-8.
std::optional<QJsonObject> strictJsonObject(const QByteArray & bytes);
