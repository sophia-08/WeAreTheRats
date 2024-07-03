#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

const std::string SOCKET_PATH = "/tmp/hid_device_socket";
const std::string LOG_FILE = "socket_communication.log";
const int RECONNECT_DELAY = 5; // seconds

std::ofstream log_file;

std::string get_current_time() {
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

void log_message(const std::string &direction, const std::string &message) {
  log_file << get_current_time() << " [" << direction << "] " << message
           << std::endl;
  log_file.flush();
}

int create_and_connect_socket() {
  while (true) {
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock == -1) {
      log_message("ERROR", "Error creating socket. Retrying in " +
                               std::to_string(RECONNECT_DELAY) + " seconds.");
      std::cerr << "Error creating socket. Retrying in " << RECONNECT_DELAY
                << " seconds." << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_DELAY));
      continue;
    }

    struct sockaddr_un server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, SOCKET_PATH.c_str(),
            sizeof(server_addr.sun_path) - 1);

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) ==
        -1) {
      log_message("ERROR", "Error connecting to socket. Retrying in " +
                               std::to_string(RECONNECT_DELAY) + " seconds.");
      std::cerr << "Error connecting to socket. Retrying in " << RECONNECT_DELAY
                << " seconds." << std::endl;
      close(sock);
      std::this_thread::sleep_for(std::chrono::seconds(RECONNECT_DELAY));
      continue;
    }

    log_message("INFO", "Successfully connected to " + SOCKET_PATH);
    std::cerr << "Successfully connected to " << SOCKET_PATH << std::endl;
    return sock;
  }
}

void handle_communication(int &sock) {
  std::vector<pollfd> fds = {{STDIN_FILENO, POLLIN, 0}, {sock, POLLIN, 0}};

  char buffer[1024];

  while (true) {
    int ret = poll(fds.data(), fds.size(), -1);
    if (ret == -1) {
      log_message("ERROR", "Error in poll");
      std::cerr << "Error in poll" << std::endl;
      break;
    }

    for (size_t i = 0; i < fds.size(); ++i) {
      if (fds[i].revents & POLLIN) {
        ssize_t bytes_read = read(fds[i].fd, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) {
          if (fds[i].fd == sock) {
            log_message("INFO", "Socket closed. Attempting to reconnect.");
            std::cerr << "Socket closed. Attempting to reconnect." << std::endl;
            close(sock);
            sock = create_and_connect_socket();
            fds[1].fd = sock;
            continue;
          } else {
            log_message("INFO", "EOF on stdin. Exiting.");
            std::cerr << "EOF on stdin. Exiting." << std::endl;
            return;
          }
        }

        buffer[bytes_read] = '\0';
        std::string message(buffer);

        if (fds[i].fd == STDIN_FILENO) {
          // Send data from stdin to socket
          ssize_t bytes_sent = send(sock, &buffer[4], bytes_read-4, 0);
          if (bytes_sent == -1) {
            log_message("ERROR",
                        "Failed to send data to socket. Reconnecting.");
            std::cerr << "Failed to send data to socket. Reconnecting."
                      << std::endl;
            close(sock);
            sock = create_and_connect_socket();
            fds[1].fd = sock;
            continue;
          }
          log_message("STDIN -> SOCKET", message);
        } else {
          // Send data from socket to stdout
          log_message("SOCKET -> STDOUT", message);
          unsigned int len = message.length();
          // We need to send the 4 bytes of length information
          std::cout << char(((len >> 0) & 0xFF)) << char(((len >> 8) & 0xFF))
                    << char(((len >> 16) & 0xFF)) << char(((len >> 24) & 0xFF));

          std::cout << message << std::flush;
        }
      }
    }
  }
}

int main() {
  log_file.open(LOG_FILE, std::ios::app);
  if (!log_file.is_open()) {
    std::cerr << "Failed to open log file" << std::endl;
    return 1;
  }

  log_message("INFO", "Program started");
  std::cerr << "Program started. Logs will be written to " << LOG_FILE
            << std::endl;

  int sock = create_and_connect_socket();

  std::cerr << "Type your messages. Press Ctrl+D to exit." << std::endl;

  handle_communication(sock);

  close(sock);
  log_message("INFO", "Program ended");
  log_file.close();

  std::cerr << "Program ended." << std::endl;

  return 0;
}