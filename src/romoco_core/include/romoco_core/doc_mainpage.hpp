#ifndef ROMOCO_DOC_MAINPAGE_HPP_
#define ROMOCO_DOC_MAINPAGE_HPP_

/// \file doc_mainpage.hpp
/// \brief Central Doxygen group definitions for the RoMoCo stack.
/// \details This file is never compiled. It exists purely for documentation.

/**
 * \mainpage RoMoCo (Reduced-Order Model-based Control) Stack
 *
 * \section intro_sec Introduction
 * RoMoCo is a modular locomotion control framework designed to bridge reduced-order 
 * modeling (ROM) with full-body robot control. It supports both research and deployment 
 * on a range of legged platforms, including Cassie, Unitree G1, and other humanoids.
 *
 * The stack provides reusable building blocks for:
 * - **Reduced-order model planning** (HLIP, MLIP, ALIP, DCM, etc.).
 * - **Whole-body embedding** of ROM trajectories into task-space outputs.
 * - **Torque-level control** through optimization-based (QP/ID) and kinematic (IK) solvers.
 * - **Hardware and simulation interfaces** for transparent deployment across MuJoCo, Unitree SDK, and ROS 2.
 *
 * RoMoCo emphasizes modularity: each package isolates a single responsibility—robot models, 
 * planners, outputs, solvers, simulation backends, or user interfaces—while providing 
 * consistent abstractions through common base classes. This allows new robots or 
 * controllers to be integrated by only extending the relevant module, without rewriting 
 * the entire stack.
 *
 * \section philosophy_sec Design Philosophy
 * The framework follows three guiding principles:
 * - **Modularity:** Each subsystem (planning, embedding, torque solving, hardware interface) 
 *   is a standalone package.
 * - **Reusability:** Core components (Pinocchio-based kinematics/dynamics, trajectory generators, 
 *   utilities) are designed to be robot-agnostic and reusable across projects.
 * - **Transparency:** Clear abstractions and extensive documentation allow researchers to focus on 
 *   algorithm design while relying on RoMoCo for low-level interfacing and consistency.
 *
 * \section modules_sec Modules
 * The main modules of RoMoCo are:
 * - \ref group_interface  
 *   Provides hardware and simulation interfaces, including MuJoCo integration and ROS 2 message transport. 
 *   These interfaces standardize the flow of state feedback and torque commands across real and simulated robots.
 *
 * - \ref group_utils  
 *   Utility functions and math tools (e.g., Bézier curves, algebraic Riccati solvers, hyperbolic functions). 
 *   These serve as the mathematical backbone for planners, controllers, and log analysis.
 *
 * - \ref group_controller  
 *   Core components that transform reduced-order plans into full-body robot actions.  
 *   Includes ROM planners, output embeddings, and torque solvers.
 *
 * - \ref group_ui  
 *   User-facing utilities such as screen radios, command interfaces, and visualization tools. 
 *   These modules support intuitive debugging, teleoperation, and runtime configuration.
 *
 * - \ref group_cassie_examples  
 *   Example configurations, controllers, and launch files for the Cassie robot. Demonstrates 
 *   both simulation and hardware deployment pipelines, serving as a template for other bipeds.
 *
 * - \ref group_g1_examples  
 *   Example configurations and interfaces for the Unitree G1 humanoid, highlighting differences 
 *   in actuator characteristics (position-controlled vs torque-controlled) and showcasing 
 *   IK-based control strategies.
 *
 * \section contrib_sec Contributing and Extending
 * RoMoCo is intended as a research toolbox. Users are encouraged to:
 * - Extend existing reduced-order planners or add new formulations.
 * - Implement custom outputs or task embeddings.
 * - Swap in new controllers or solvers while retaining the same robot interface.
 * - Leverage simulation/hardware transparency to benchmark algorithms across multiple robots.
 *
 * Through its modular design, RoMoCo enables rapid experimentation and reproducible research 
 * in bipedal and humanoid locomotion.
 */

/// \defgroup group_utils Utils
/// @brief Utility functions and math tools.
/// Includes algebraic solvers, Bézier curve generators, and general-purpose helpers
/// used across planners, controllers, and log analysis.

/// \defgroup group_controller Controllers
/// @brief Core components that bridge ROM planning with whole-body robot control.
/// Organized into the following subgroups:
/// - \ref group_robot
/// - \ref group_ro_planner  
/// - \ref group_output  
/// - \ref group_solver  

/// \defgroup group_robot Robot Models
/// @ingroup group_controller
/// Pinocchio-based robot models with kinematics, dynamics, and contact definitions.

/// \defgroup group_ro_planner Reduced-Order Planners
/// @ingroup group_controller
/// Canonical locomotion models such as LIP, HLIP, MLIP, ALIP, and DCM.
/// Generate high-level trajectories for COM motion, step timing, and foot placement.

/// \defgroup group_output Output Embedding
/// @ingroup group_controller
/// Map reduced-order trajectories into task-space or joint-space references 
/// (e.g., swing foot trajectories, pelvis orientation, contact-consistent outputs).

/// \defgroup group_solver Torque Solvers
/// @ingroup group_controller
/// Optimization and control backends for realizing outputs subject to physical constraints.
/// Includes:
/// - Quadratic Programming (QP) solvers
/// - Inverse Dynamics (ID) solvers
/// - Inverse Kinematics (IK) solvers

/// \defgroup group_interface Interfaces
/// @brief Robot interfaces and system integration.
/// Provides simulation and hardware interfaces (MuJoCo, Unitree SDK), 
/// ROS 2 message transport, and locomotion state machines.

/// \defgroup group_ui UI
/// @brief User interfaces for command and visualization.
/// Includes screen radios, ROS 2 nodes, and GUI utilities for runtime control.

/// \defgroup group_cassie_examples Example_CassieStacks
/// @brief End-to-end robot stacks for Cassie.
/// Demonstrates simulation and hardware pipelines, serving as a template for other bipeds.

/// \defgroup group_g1_examples Example_G1Stacks
/// @brief End-to-end robot stacks for Unitree G1.
/// Highlights differences in actuator characteristics and showcases IK-based control strategies.

#endif  // ROMOCO_DOC_MAINPAGE_HPP_
