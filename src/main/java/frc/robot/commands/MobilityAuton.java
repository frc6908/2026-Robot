// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * A simple autonomous routine that just drives forward to get the "mobility" bonus.
 *
 * In FRC, you get bonus points if your robot moves out of its starting zone during
 * autonomous. This is the simplest possible auto -- it just resets the heading and
 * drives forward slowly for 10 seconds.
 *
 * This is a "utility class" (all static methods, no instances), which is why the
 * constructor throws an exception -- you're not supposed to create an object from it,
 * just call MobilityAuton.exampleAuto().
 *
 * Note: this auto is mostly replaced by PathPlanner-based autos now, but it's kept
 * around as a fallback.
 */
public final class MobilityAuton {

  /**
   * Creates a sequential auto command:
   *   1. Reset the NavX heading (1 second timeout)
   *   2. Drive forward slowly (-0.3 speed) for 10 seconds
   *
   * Commands.sequence() runs commands one after another -- the second one
   * doesn't start until the first one finishes (or times out).
   *
   * WANT TO CHANGE how fast/far the robot drives in this auto?
   *   - Change -0.3 to a different value (more negative = faster forward)
   *   - Change .withTimeout(10) to a different number of seconds
   *
   * WANT TO BUILD a more complex auto?
   *   Use PathPlanner (the GUI tool) instead -- it's much easier than writing
   *   code for complex paths. See the .auto and .path files in
   *   src/main/deploy/pathplanner/
   *
   * @param m_drivetrain  the swerve subsystem to drive with
   * @return  a Command that can be scheduled by the autonomous system
   */
  public static Command exampleAuto(SwerveSubsystem m_drivetrain) {
    return Commands.sequence(
      new ResetNavX(m_drivetrain).withTimeout(1),
      // The negative value (-0.3) means "forward" because the joystick Y axis is inverted
      // (pushing the stick forward gives a negative value)
      new SwerveJoystickCmd(m_drivetrain,  () -> -0.3, () -> 0, () -> 0, () -> 0).withTimeout(10)
    );
  }

  // Private constructor prevents creating instances of this utility class
  private MobilityAuton() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
