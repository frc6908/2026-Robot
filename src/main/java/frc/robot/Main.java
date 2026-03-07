// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The entry point for the robot program. This is the very first thing that runs
 * when the robot boots up (or when you start a simulation). It tells WPILib to
 * start the Robot class.
 *
 * Think of this like the "main()" in any normal Java program -- it's the
 * starting line. WPILib takes over from here and calls the appropriate methods
 * in Robot.java depending on the robot's current mode.
 *
 * You almost never need to touch this file. The only reason you'd change it is
 * if you renamed the Robot class (which you probably shouldn't do).
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
