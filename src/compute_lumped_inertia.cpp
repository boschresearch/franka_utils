#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

/*!
 * Returns skew-symmetric matrix for a given vector.
 *
 * @tparam Base Base type of Eigen vector.
 * @param v 3 vector.
 * @returns m 3x3 matrix.
 */
template <typename Base>
Eigen::Matrix3d
skew(const Eigen::MatrixBase<Base> &v)
{
  static_assert(Base::SizeAtCompileTime == 3, "Skew Matrix has wrong dimensions");
  Eigen::Matrix3d m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

/*!
 * Lump inertia parameters until end of tree.
 * @param link Current link consired in the recursion.
 * @param robot URDF model of robot.
 * @param T Current transformation matrix.
 * @param m Lumped mass (output, will not be initialized).
 * @param l Lumped mass * com (output, will not be initialized).
 * @param I Lumped inertia tensor (output, will not be initialized).
 */
template <typename l_t, typename I_t>
void
compute_lumped_inertia_impl(urdf::LinkConstSharedPtr link,
                            const urdf::ModelInterface &robot,
                            const Eigen::Matrix4d &T,
                            double &m,
                            l_t &l,
                            I_t &I)
{
  // Read inertia properties of link
  if (link->inertial) {
    // Parse data from URDF into Eigen objects
    const auto &il = *link->inertial;
    /// mass of link
    double dm = il.mass;
    /// reference to position of inertia frame relative to link frame
    const auto &iopr = il.origin.position;
    /// reference to orientation of inertia frame relative to link frame
    const auto &ioqr = il.origin.rotation;
    /// Eigen vector for position of inertia frame relative to link frame
    auto iop = (Eigen::Vector3d() << iopr.x, iopr.y, iopr.z).finished();
    /// Eigen quaternion for orientation of inertia frame relative to link frame
    Eigen::Quaterniond ioq(ioqr.w, ioqr.x, ioqr.y, ioqr.z);
    /// Eigen matrix for homogeneous transformation matrix of inertia frame
    /// relative to link frame (iot, iot, iot, ...)
    Eigen::Matrix4d iot = Eigen::Matrix4d::Identity();
    iot.template block<3, 1>(0, 3) = iop;
    iot.template block<3, 3>(0, 0) = ioq.normalized().toRotationMatrix();
    /// Mass moment of inertia tensor
    auto Ilink =
      (Eigen::Matrix3d() << il.ixx, il.ixy, il.ixz, il.ixy, il.iyy, il.iyz, il.ixz, il.iyz, il.izz)
        .finished();

    // Compute transformation from "root frame" to inertia frame
    Eigen::Matrix4d iot_root = T * iot;

    // Previous COM (needed for parallel axis theorem)
    double m_prev = m;
    Eigen::Vector3d com_prev = Eigen::Vector3d::Zero();
    if (m > 0) {
      com_prev = l / m;
    }

    // Add mass (0-th moment)
    m += dm;

    // Add mass*com (1-st moment)
    Eigen::Vector3d iop_root = iot_root.template block<3, 1>(0, 3);
    l += iop_root * dm;

    // Composite COM (needed for parallel axis theorem)
    Eigen::Vector3d com_new = Eigen::Vector3d::Zero();
    if (m > 0) {
      com_new = l / m;
    }

    // Update I
    /// Correct for shifted center of mass
    I -= m_prev * skew(com_prev - com_new) * skew(com_prev - com_new);
    /// Add contribution of new term
    Eigen::Matrix3d ior_root = iot_root.template block<3, 3>(0, 0);
    I += ior_root * Ilink * ior_root.transpose() -
         dm * skew(iop_root - com_new) * skew(iop_root - com_new);
  }

  // Explore all child links
  for (auto child_joint : link->child_joints) {
    auto child_link = robot.getLink(child_joint->child_link_name);

    // Compute relative pose of child link
    /// reference to child link position with respect to link frame
    const auto &cpr = child_joint->parent_to_joint_origin_transform.position;
    /// reference to child link orientation with respect to link frame
    const auto &crr = child_joint->parent_to_joint_origin_transform.rotation;
    /// Eigen vector for position of child link with respect to link frame
    auto cp = (Eigen::Vector3d() << cpr.x, cpr.y, cpr.z).finished();
    /// Eigen quaternion for orientation of child link with respect to link
    /// frame
    Eigen::Quaterniond cq(crr.w, crr.x, crr.y, crr.z);
    /// Eigen matrix for homogeneous transformation matrix of child link with
    /// respect to link frame
    Eigen::Matrix4d ct = Eigen::Matrix4d::Identity();
    ct.template block<3, 1>(0, 3) = cp;
    ct.template block<3, 3>(0, 0) = cq.normalized().toRotationMatrix();

    // Compute pose of child link with respect to root frame
    Eigen::Matrix4d Tnext = T * ct;

    // Recurse...
    compute_lumped_inertia_impl(child_link, robot, Tnext, m, l, I);
  }
}

/*!
 * Lump inertia parameters until end of tree.
 * @param link Current link consired in the recursion.
 * @param robot URDF model of robot.
 * @param T Current transformation matrix.
 * @param m Lumped mass (output).
 * @param com Lumped center of mass (output).
 * @param I Lumped inertia tensor (output).
 */
template <typename com_t, typename I_t>
bool
compute_lumped_inertia(urdf::LinkConstSharedPtr link,
                       const urdf::ModelInterface &robot,
                       const Eigen::Matrix4d &T,
                       double &m,
                       com_t &com,
                       I_t &I)
{
  m = 0;
  I = Eigen::Matrix3d::Zero();

  Eigen::Vector3d l = Eigen::Vector3d::Zero();
  compute_lumped_inertia_impl(link, robot, T, m, l, I);
  if (m > 0) {
    com = l / m;
  } else {
    com = Eigen::Vector3d::Zero();
  }

  // Sanity checks
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> I_eigen_solver(I, false);
  auto I_eig = I_eigen_solver.eigenvalues();
  /// Mass must be positive
  if (m < 0) {
    std::cerr << "compute_lumped_inertia: Lumped mass is negative." << std::endl;
    return false;
  }
  /// Inertia tensor must be symmetric
  if (std::abs(I(1, 0) - I(0, 1)) + std::abs(I(2, 0) - I(0, 2)) + std::abs(I(1, 2) - I(2, 1)) >
      std::numeric_limits<double>::epsilon())
  {
    std::cerr << "compute_lumped_inertia: Lumped inertia tensor is not symmetric." << std::endl;
    return false;
  }
  /// Inertia tensor must be positive definite
  if ((I_eig(0) < 0) || (I_eig(1) < 0) || (I_eig(2) < 0)) {
    std::cerr << "compute_lumped_inertia: Lumped inertia tensor is not "
                 "positive semi-definite."
              << std::endl;
    return false;
  }
  /// Inertia tensor must satisfy triangle inequality
  if ((I_eig(0) + I_eig(1) < I_eig(2)) || (I_eig(0) + I_eig(2) < I_eig(1)) ||
      (I_eig(1) + I_eig(2) < I_eig(0)))
  {
    std::cerr << "compute_lumped_inertia: Lumped inertia tensor does not "
                 "satisfy triangle inequality."
              << std::endl;
    return false;
  }

  // Enforce exact symmetry of inertia tensor
  I(1, 0) = I(0, 1);
  I(2, 0) = I(0, 2);
  I(2, 1) = I(1, 2);

  return true;
}

int
main(int argc, char *argv[])
{
  if (argc < 2 || argc > 3) {
    std::cout << "usage '" << argv[0] << " $URDF_PATH $ROBOT_IP'" << std::endl;
    return EXIT_FAILURE;
  }

  std::unique_ptr<franka::Robot> robot = nullptr;

  if (argc == 3) {
    std::cout << "connecting to: " << argv[2] << std::endl;
    try {
      robot = std::make_unique<franka::Robot>(argv[2]);
    } catch (const franka::NetworkException &e) {
      robot = nullptr;
      std::cerr << "Franka not reachable: " << e.what() << std::endl;
    }
  }

  if (robot) {
    const franka::RobotState robot_state = robot->readOnce();
    std::cout << "Franka current state: " << std::endl << robot_state << std::endl;
  }

  // parse URDF as file path or as content string
  std::shared_ptr<urdf::ModelInterface> urdf_model = nullptr;
  // try by file path
  urdf_model = urdf::parseURDFFile(argv[1]);
  // try by content string
  if (!urdf_model)
    urdf_model = urdf::parseURDF(argv[1]);
  if (!urdf_model) {
    std::cerr << "error reading: " << argv[1] << std::endl;
    return EXIT_FAILURE;
  }
  urdf::LinkConstSharedPtr root_link =
    urdf_model->getLink("panda_link8"); /// Flange (has no inertia info)

  Eigen::Matrix4d T0 = Eigen::Matrix4d::Identity();
  double load_mass = 0;
  std::array<double, 3> F_x_Cload;
  std::array<double, 9> load_inertia;
  Eigen::Map<Eigen::Vector3d> com(F_x_Cload.data());
  Eigen::Map<Eigen::Matrix3d> I(load_inertia.data());

  if (compute_lumped_inertia(root_link, *urdf_model, T0, load_mass, com, I)) {
    std::cout << "URDF load mass: " << load_mass << std::endl;
    Eigen::Map<Eigen::Matrix3d> load_inertia__(load_inertia.data());
    std::cout << "URDF load inertia: " << std::endl << load_inertia__ << std::endl;
    Eigen::Map<Eigen::Vector3d> F_x_Cload__(F_x_Cload.data());
    std::cout << "URDF load CoM: " << F_x_Cload__.transpose() << std::endl;

    if (robot) {
      robot->setLoad(load_mass, F_x_Cload, load_inertia);
    }
    return EXIT_SUCCESS;
  } else {
    std::cerr << "Could not set lumped end-effector inertia. Shutting down ..." << std::endl;
    return EXIT_FAILURE;
  }
}
