#include "strict_json.hpp"
#include <QJsonArray>
#include <QJsonDocument>
#include <QSet>

namespace {
class Scanner
{
public:
  explicit Scanner(const QByteArray & bytes) : data_(bytes) {}
  bool complete() {skip(); return value(0) && (skip(), i_ == data_.size());}
private:
  void skip() {while (i_ < data_.size() && (data_[i_] == ' ' || data_[i_] == '\n' || data_[i_] == '\r' || data_[i_] == '\t')) {++i_;}}
  bool string(QString * decoded = nullptr)
  {
    if (i_ >= data_.size() || data_[i_] != '"') {return false;}
    const int start = i_++;
    while (i_ < data_.size()) {
      const char ch = data_[i_++];
      if (ch == '\\') {if (i_ >= data_.size()) {return false;} ++i_;}
      else if (ch == '"') {
        if (decoded) {
          const auto doc = QJsonDocument::fromJson("[" + data_.mid(start, i_ - start) + "]");
          if (!doc.isArray() || !doc.array().at(0).isString()) {return false;}
          *decoded = doc.array().at(0).toString();
        }
        return true;
      }
    }
    return false;
  }
  bool value(int depth)
  {
    skip();
    if (depth > 64 || i_ >= data_.size()) {return false;}
    if (data_[i_] == '"') {return string();}
    if (data_[i_] == '{') {
      ++i_; skip(); QSet<QString> keys;
      if (i_ < data_.size() && data_[i_] == '}') {++i_; return true;}
      while (i_ < data_.size()) {
        QString key;
        if (!string(&key) || keys.contains(key)) {return false;}
        keys.insert(key); skip();
        if (i_ >= data_.size() || data_[i_++] != ':' || !value(depth + 1)) {return false;}
        skip();
        if (i_ < data_.size() && data_[i_] == '}') {++i_; return true;}
        if (i_ >= data_.size() || data_[i_++] != ',') {return false;}
        skip();
      }
      return false;
    }
    if (data_[i_] == '[') {
      ++i_; skip();
      if (i_ < data_.size() && data_[i_] == ']') {++i_; return true;}
      while (i_ < data_.size()) {
        if (!value(depth + 1)) {return false;}
        skip();
        if (i_ < data_.size() && data_[i_] == ']') {++i_; return true;}
        if (i_ >= data_.size() || data_[i_++] != ',') {return false;}
      }
      return false;
    }
    const int start = i_;
    while (i_ < data_.size() && data_[i_] != ',' && data_[i_] != ']' && data_[i_] != '}' &&
      data_[i_] != ' ' && data_[i_] != '\t' && data_[i_] != '\r' && data_[i_] != '\n') {++i_;}
    return i_ > start;
  }
  const QByteArray & data_;
  int i_ = 0;
};
}

std::optional<QJsonObject> strictJsonObject(const QByteArray & bytes)
{
  if (bytes.isEmpty() || bytes.size() > 256 * 1024 || QString::fromUtf8(bytes).toUtf8() != bytes) {return std::nullopt;}
  const auto doc = QJsonDocument::fromJson(bytes);
  if (!doc.isObject() || !Scanner(bytes).complete()) {return std::nullopt;}
  return doc.object();
}
