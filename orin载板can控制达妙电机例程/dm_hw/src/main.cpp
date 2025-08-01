#include "control_loop.h"
#include <csignal>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}


int main(int argc, char** argv)
{
  std::signal(SIGINT, signalHandler);

  try {
      std::shared_ptr<damiao::DmHW> motor_dm_hw = std::make_shared<damiao::DmHW>();

      damiao::DmHWLoop control_loop(motor_dm_hw);

      while (running) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "The program exited safely." << std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
