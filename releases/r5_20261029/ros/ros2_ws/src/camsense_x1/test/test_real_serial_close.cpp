// Copyright 2024 rossihwang@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <serial/serial.h>
#if defined(__linux__)
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <iostream>
#include <string>
#include <exception>
#include <cstdlib>

void child_close_failure()
{
  std::set_terminate(
    []() {
      std::cerr << "TERMINATE_CALLED\n";
      std::exit(86);
    });

  int master, slave;
  if (openpty(&master, &slave, NULL, NULL, NULL) == -1) {
    std::cerr << "openpty failed\n";
    std::exit(1);
  }
  char name_buf[256];
  if (ttyname_r(slave, name_buf, sizeof(name_buf)) != 0) {
    std::cerr << "ttyname_r failed\n";
    std::exit(1);
  }
  std::string port_name(name_buf);

  bool state_closed = false;

  // Scoped block to trigger destructor
  {
    serial::Serial port(port_name, 115200, serial::Timeout::simpleTimeout(100));
    if (!port.isOpen()) {
      std::cerr << "port not open\n";
      std::exit(2);
    }

    // Sabotage the exact file descriptor opened by serial implementation
    DIR * dir = opendir("/proc/self/fd");
    if (!dir) {
      std::cerr << "Failed to open /proc/self/fd\n";
      std::exit(1);
    }

    int serial_fd = -1;
    struct dirent * ent;
    while ((ent = readdir(dir)) != NULL) {
      if (ent->d_name[0] == '.') {
        continue;
      }
      int fd = std::atoi(ent->d_name);
      if (fd == master || fd == slave || fd == dirfd(dir)) {
        continue;
      }

      char fd_name_buf[256];
      if (ttyname_r(fd, fd_name_buf, sizeof(fd_name_buf)) == 0) {
        if (std::string(fd_name_buf) == port_name) {
          serial_fd = fd;
          break;
        }
      }
    }
    closedir(dir);

    if (serial_fd == -1) {
      std::cerr << "Could not find exact FD for serial port\n";
      std::exit(1);
    }
    ::close(serial_fd);

    try {
      port.close();
      // Expected to throw because we closed its fd behind its back
      std::cerr << "explicit close succeeded unexpectedly\n";
      std::exit(5);
    } catch (const serial::IOException & e) {
      std::cerr << "FIRST_CLOSE_CAUGHT: " << e.what() << "\n";
    }

    // Capture state
    state_closed = !port.isOpen();
  }

  if (!state_closed) {
    std::cerr << "port still open after failed close\n";
    std::exit(3);
  }

  // Destructor returned normally without throwing std::terminate
  std::cerr << "REAL_SERIAL_DESTRUCTOR_RETURNED\n";
  std::exit(0);
}

TEST(SerialDestructor, CloseFailureIsExceptionSafe)
{
  EXPECT_EXIT(
    child_close_failure(), testing::ExitedWithCode(0),
    "FIRST_CLOSE_CAUGHT:.*REAL_SERIAL_DESTRUCTOR_RETURNED");
}

void child_normal_lifecycle()
{
  std::set_terminate(
    []() {
      std::cerr << "TERMINATE_CALLED\n";
      std::exit(86);
    });

  int master, slave;
  if (openpty(&master, &slave, NULL, NULL, NULL) == -1) {
    std::exit(1);
  }
  char name_buf[256];
  if (ttyname_r(slave, name_buf, sizeof(name_buf)) != 0) {
    std::exit(1);
  }
  std::string port_name(name_buf);

  for (int i = 0; i < 25; ++i) {
    {
      serial::Serial port(port_name, 115200, serial::Timeout::simpleTimeout(100));
      if (!port.isOpen()) {
        std::exit(2);
      }
      port.close();
      if (port.isOpen()) {
        std::exit(3);
      }
      port.open();
      if (!port.isOpen()) {
        std::exit(4);
      }
    }
  }
  std::cerr << "NORMAL_LIFECYCLE_25_PASS\n";
  std::exit(0);
}

TEST(SerialDestructor, NormalLifecycle)
{
  EXPECT_EXIT(
    child_normal_lifecycle(), testing::ExitedWithCode(0),
    "NORMAL_LIFECYCLE_25_PASS");
}

#endif  // __linux__
