#include <osc_controller/OSC_Controller.hpp>

namespace osc_controller {

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

////////////////////////////////////////////////////////////////////////////////
// Admittance controller for Whole body controller
Eigen::VectorXd OSC_Controller::admittance_controller(Eigen::VectorXd torque_out){

    Eigen::MatrixXd damp_des_     = Eigen::MatrixXd::Identity(3, 3) ;
    damp_des_.topLeftCorner(2, 2) = admittance_linear_damping*Eigen::MatrixXd::Identity(2, 2); // Liner vel damping
    damp_des_(2,2) = admittance_angular_damping ; // Angular vel damping 

    Eigen::MatrixXd inertia_des = 1.0*Eigen::MatrixXd::Identity(3, 3);

    damp_des_ = damp_des_.inverse();

    Eigen::VectorXd mob_base_tor = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd mob_base_vel = Eigen::VectorXd::Zero(3);

    mob_base_tor << torque_out(0), torque_out(1), torque_out(2);
    //std::cout << "Tau mobile base : \n" << mob_base_tor << std::endl;

    mob_base_vel = damp_des_*mob_base_tor - damp_des_*admittance_desired_inertia*Eigen::VectorXd::Zero(3);
    
    return mob_base_vel;
}

} // end namespace