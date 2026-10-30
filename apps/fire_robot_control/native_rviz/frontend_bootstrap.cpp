#include "frontend_bootstrap.hpp"

#include <QByteArray>
#include <QDir>
#include <QRegularExpression>
#include <QUuid>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

std::optional<FrontendBootstrap> readFrontendBootstrap(int fd)
{
  if (fd < 3) {return std::nullopt;}
  struct stat info{};
  const int flags = fcntl(fd, F_GETFL);
  if (flags < 0 || (flags & O_ACCMODE) != O_RDONLY || fstat(fd, &info) != 0 ||
    !S_ISREG(info.st_mode) || info.st_uid != getuid() || (info.st_mode & 0077) != 0 ||
    info.st_size <= 0 || info.st_size > 4096)
  {
    return std::nullopt;
  }
  QByteArray raw(static_cast<int>(info.st_size), '\0');
  if (pread(fd, raw.data(), static_cast<size_t>(raw.size()), 0) != raw.size()) {
    return std::nullopt;
  }
  const auto parts = raw.split('\n');
  if (parts.size() != 7 || parts[0] != "1" || !parts[6].isEmpty()) {
    return std::nullopt;
  }
  const QString session = QString::fromLatin1(parts[1]);
  const QUuid uuid(session);
  const QRegularExpression hex32("^[0-9a-f]{32}$"), hex64("^[0-9a-f]{64}$");
  const QString generation = QString::fromLatin1(parts[2]);
  const QString nonce = QString::fromLatin1(parts[3]);
  const QString packet = QString::fromLatin1(parts[4]);
  if (uuid.isNull() || uuid.toString(QUuid::WithoutBraces) != session ||
    !hex32.match(generation).hasMatch() || !hex32.match(nonce).hasMatch() ||
    !hex64.match(packet).hasMatch())
  {
    return std::nullopt;
  }
  const auto decoded = QByteArray::fromBase64(parts[5]);
  const QString socket = QString::fromUtf8(decoded);
  if (decoded.isEmpty() || decoded.contains('\0') || decoded.toBase64() != parts[5] ||
    socket.toUtf8() != decoded || !QDir::isAbsolutePath(socket))
  {
    return std::nullopt;
  }
  return FrontendBootstrap{session, generation, nonce, packet, socket};
}
