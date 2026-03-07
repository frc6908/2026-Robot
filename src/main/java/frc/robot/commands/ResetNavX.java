package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Resets the NavX gyroscope heading to zero.
 *
 * After pressing this, the robot's current facing direction becomes the new "forward"
 * (0 degrees). This is useful when:
 *   - The robot's heading has drifted over time (gyro drift)
 *   - You want to re-zero the heading to match the robot's physical orientation
 *   - Field-relative driving feels "off" because the gyro's idea of "forward"
 *     doesn't match reality
 *
 * Typically pressed at the start of a match or whenever field-relative driving
 * feels misaligned.
 *
 * Bound to: Driver controller Y button (whileTrue).
 */
public class ResetNavX extends Command {
    private final SwerveSubsystem m_drivetrain;

    public ResetNavX(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Resets the gyro heading -- current direction becomes 0 degrees. */
    @Override
    public void execute() {
        m_drivetrain.resetHeading();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
