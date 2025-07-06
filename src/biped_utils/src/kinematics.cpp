#include "biped_core/kinematics.hpp"

void Kinematics1D::print() const
{
   std::cout << "Position=[" << position.transpose() << "]';\n";
   std::cout << "Velocity=[" << velocity.transpose() << "]';\n";
   std::cout << "Jacobian=[" << jacobian << "];\n";
   std::cout << "dJdq=[" << dJdq.transpose() << "]';\n";
}

void Kinematics3D::print() const
{
   std::cout << "Position=[" << position.transpose() << "]';\n";
   std::cout << "Velocity=[" << velocity.transpose() << "]';\n";
   std::cout << "Jacobian=[" << jacobian << "];\n";
   std::cout << "dJdq=[" << dJdq.transpose() << "]';\n";
}
