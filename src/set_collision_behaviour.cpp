#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>


// default values
// https://github.com/frankaemika/libfranka/blob/0.10.0/examples/examples_common.cpp#L12-L20
// clang-format off
struct {
  // torque thresholds [Nm]
  const std::array<double, 7> lower_torque_thresholds_acceleration  = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
  const std::array<double, 7> upper_torque_thresholds_acceleration  = lower_torque_thresholds_acceleration;
  const std::array<double, 7> lower_torque_thresholds_nominal       = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  const std::array<double, 7> upper_torque_thresholds_nominal       = lower_torque_thresholds_nominal;
  // force thresholds [N]
  const std::array<double, 6> lower_force_thresholds_acceleration   = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
  const std::array<double, 6> upper_force_thresholds_acceleration   = lower_force_thresholds_acceleration;
  const std::array<double, 6> lower_force_thresholds_nominal        = {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
  const std::array<double, 6> upper_force_thresholds_nominal        = lower_force_thresholds_nominal;
} defaults;
// clang-format on


template <typename T, std::size_t N>
std::array<T, N>
multiply(const T &multiplier, const std::array<T, N> &array)
{
  std::array<T, N> out;
  for (std::size_t i = 0; i < N; i++)
    out[i] = multiplier * array[i];
  return out;
}


int
main(int argc, char *argv[])
{
  if (argc != 3) {
    std::cout << "usage '" << argv[0] << " $ROBOT_IP $THRESHOLD_SCALE'" << std::endl;
    return EXIT_FAILURE;
  }

  std::unique_ptr<franka::Robot> robot = nullptr;

  std::cout << "connecting to: " << argv[1] << std::endl;
  try {
    franka::Robot robot(argv[1]);
    // scale default values
    const double threshold_scale = std::stod(argv[2]);
    robot.setCollisionBehavior(
      multiply(threshold_scale, defaults.lower_torque_thresholds_acceleration),
      multiply(threshold_scale, defaults.upper_torque_thresholds_acceleration),
      multiply(threshold_scale, defaults.lower_torque_thresholds_nominal),
      multiply(threshold_scale, defaults.upper_torque_thresholds_nominal),
      multiply(threshold_scale, defaults.lower_force_thresholds_acceleration),
      multiply(threshold_scale, defaults.upper_force_thresholds_acceleration),
      multiply(threshold_scale, defaults.lower_force_thresholds_nominal),
      multiply(threshold_scale, defaults.upper_force_thresholds_nominal));
    std::cout << "set F/T thresholds to '" << threshold_scale << "' x defaults" << std::endl;
    return EXIT_SUCCESS;
  } catch (const franka::NetworkException &e) {
    std::cerr << "Franka not reachable: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
