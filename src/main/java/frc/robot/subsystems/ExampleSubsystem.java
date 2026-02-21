// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * !! THIS IS BOILERPLATE / TEMPLATE CODE FROM WPILIB !!
 *
 * This file is NOT connected to any real hardware on the robot. Changing code
 * in here will NOT affect how the robot drives, intakes, or does anything else.
 *
 * It exists as a reference to show you what a subsystem looks like. If you want
 * to see how a real subsystem works, look at:
 *   - AlgaeMechanism.java  (simpler -- good starting point)
 *   - SwerveSubsystem.java (more complex -- the full drivetrain)
 *
 * If you're creating a NEW subsystem (like a climber or a shooter), you can copy
 * this file as a skeleton and fill in the blanks, or copy AlgaeMechanism.java
 * for a more complete starting point.
 */
public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
