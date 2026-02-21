// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This is the very first thing that runs when the robot turns on.
 * Think of it like the "main()" in any Java program -- it's the entry point.
 *
 * All it does is tell WPILib "hey, start our Robot class." You almost never
 * need to touch this file.
 */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    // This hands control over to WPILib, which will create our Robot object
    // and start calling its lifecycle methods (robotInit, teleopPeriodic, etc.)
    RobotBase.startRobot(Robot::new);
  }
}
