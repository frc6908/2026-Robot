// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is the main robot class. WPILib calls different methods here depending
 * on what mode the robot is in (disabled, autonomous, teleop, test).
 *
 * TimedRobot means WPILib calls our "periodic" methods every 20 milliseconds
 * (50 times per second). That's the robot's heartbeat -- every 20ms it checks
 * for new joystick input, updates motors, reads sensors, etc.
 *
 * Most of our actual robot logic lives in RobotContainer (subsystems + commands),
 * not here. This file is mostly just plumbing that connects WPILib to our code.
 */
public class Robot extends TimedRobot {
  // Stores whatever autonomous command we pick from the dashboard
  private Command m_autonomousCommand;

  // RobotContainer is where all our subsystems and button bindings live
  private final RobotContainer m_robotContainer;

  /**
   * Constructor -- runs once when the robot first boots up.
   */
  public Robot() {
    // Create our RobotContainer, which sets up all subsystems, button bindings,
    // and the autonomous chooser on the dashboard
    m_robotContainer = new RobotContainer();
  }

  /**
   * Runs every 20ms no matter what mode we're in (disabled, auto, teleop, etc.).
   * The most important thing here is running the CommandScheduler -- that's the
   * engine that makes the whole command-based framework work.
   */
  @Override
  public void robotPeriodic() {
    // The CommandScheduler is the brain of command-based programming.
    // Every 20ms it:
    //   1. Checks if any buttons were pressed (triggers)
    //   2. Starts new commands that were just scheduled
    //   3. Runs the execute() method of all active commands
    //   4. Removes commands that are finished
    //   5. Calls periodic() on every subsystem
    CommandScheduler.getInstance().run();
  }

  // --- Disabled Mode ---
  // These run when the robot is on but not enabled (e.g. before a match)
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  // --- Autonomous Mode ---

  /**
   * Runs once at the start of autonomous. Grabs whichever auto routine
   * was selected on the dashboard and starts it.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  // --- Teleop Mode (driver-controlled) ---

  /**
   * Runs once at the start of teleop. Cancels the auto command so it doesn't
   * keep running while drivers are trying to control the robot.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  // --- Test Mode ---
  @Override
  public void testInit() {
    // Cancel everything so we start with a clean slate in test mode
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  // --- Simulation (for testing on your laptop without a real robot) ---
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
