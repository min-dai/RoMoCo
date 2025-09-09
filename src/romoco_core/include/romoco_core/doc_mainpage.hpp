#ifndef ROMOCO_DOC_MAINPAGE_HPP_
#define ROMOCO_DOC_MAINPAGE_HPP_

/// \file doc_mainpage.hpp
/// \brief Central Doxygen group definitions for the RoMoCo stack.
/// \details This file is never compiled. It exists purely for documentation.

/**
 * \mainpage RoMoCo (Reduced-Order Model-based Control) Stack
 *
 * \section intro_sec Introduction
 * RoMoCo is a modular locomotion/control toolbox for Cassie, G1, and other bipeds,
 * organized into interfaces, outputs, torque solvers, and example robot stacks.
 *
 * \section modules_sec Modules
 * - \ref group_core
 * - \ref group_interface
 * - \ref group_utils
 * - \ref group_output
 * - \ref group_torque
 * - \ref group_examples
 */

/// \defgroup group_core Core
/// Core types, constants, math helpers.

/// \defgroup group_interface Interfaces
/// Hardware and simulation interfaces, ROS2 transport.

/// \defgroup group_utils Utils
/// Utility functions (e.g., algebraic Riccati solver).

/// \defgroup group_output Outputs
/// Output classes (HLIP, MLIP, MLIP_NLP), planners, tracking signals.

/// \defgroup group_torque TorqueSolvers
/// Torque controllers (PD, Velocity-IK, TSC/QP).

/// \defgroup group_examples ExampleStacks
/// End-to-end robot stacks (e.g., Unitree G1, Cassie).

#endif  // ROMOCO_DOC_MAINPAGE_HPP_
