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
 * - \ref group_interface
 * - \ref group_utils
 * - \ref group_controller
 * - \ref group_ui
 * - \ref group_cassie_examples
 * - \ref group_g1_examples
 */


/// \defgroup group_utils Utils
/// Utility functions (e.g., algebraic Riccati solver).

/// \defgroup group_controller Controllers
/// RO planners, Output embedding classes, torque solvers.

/// \defgroup group_interface Interfaces
/// Robot interfaces (e.g., MuJoCo, Hardware).
/// State machines and desired command interpreters.

/// \defgroup group_ui UI
/// GUI and ROS2 nodes for radio slider control.

/// \defgroup group_cassie_examples ExampleCassieStacks
/// End-to-end robot stacks for Cassie

/// \defgroup group_g1_examples ExampleG1Stacks
/// End-to-end robot stacks for G1


#endif  // ROMOCO_DOC_MAINPAGE_HPP_
