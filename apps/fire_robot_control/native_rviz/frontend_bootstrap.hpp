#pragma once

#include <QString>
#include <optional>

// The caller must receive the FD from its trusted session supervisor. Its
// presence alone does not prove the R5 packet, operator or robot is ready.
struct FrontendBootstrap
{
  QString session;
  QString generation;
  QString nonce;
  QString packet;
  QString socketPath;
};

std::optional<FrontendBootstrap> readFrontendBootstrap(int fd);
