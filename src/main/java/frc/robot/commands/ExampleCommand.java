// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * !! THIS IS BOILERPLATE / TEMPLATE CODE FROM WPILIB !!
 *
 * This command doesn't do anything -- its execute() method is empty. Changing
 * code in here will NOT affect how the robot behaves.
 *
 * It exists as a reference to show you the structure of a command (the lifecycle
 * methods: initialize, execute, end, isFinished). If you want to see how a real
 * command works, look at:
 *   - IntakeAlgae.java  (simplest real command -- just sets a motor speed)
 *   - MoveArm.java      (slightly more complex -- handles direction)
 *   - SwerveJoystickCmd.java  (the most complex -- reads joystick input and drives)
 *
 * If you're creating a NEW command, you can copy this file as a skeleton and fill
 * in the blanks, or copy IntakeAlgae.java for a more complete starting point.
 */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
