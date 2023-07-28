#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>


int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "usage '" << argv[0] << " $ROBOT_IP $THRESHOLD'" << std::endl;
    return EXIT_FAILURE;
  }

  std::unique_ptr<franka::Robot> robot = nullptr;

  std::cout << "connecting to: " << argv[1] << std::endl;
  try {
    franka::Robot robot(argv[1]);
    const double threshold = std::stod(argv[2]);
    robot.setCollisionBehavior(
      {{threshold, threshold, threshold, threshold, threshold, threshold, threshold}},  // contact torque thresholds for each joint [Nm]
      {{threshold, threshold, threshold, threshold, threshold, threshold, threshold}},  // collision torque thresholds for each joint [Nm]
      {{threshold, threshold, threshold, threshold, threshold, threshold}},             // contact force thresholds [N]
      {{threshold, threshold, threshold, threshold, threshold, threshold}}              // collision force thresholds [N]
    );
    return EXIT_SUCCESS;
  } catch (const franka::NetworkException &e) {
    std::cerr << "Franka not reachable: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
