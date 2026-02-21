package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Resets the NavX gyroscope heading to 0 degrees.
 *
 * After running this command, whatever direction the robot is currently facing
 * becomes the new "forward" (0 degrees). This is useful when:
 *   - The robot's heading drifts over time
 *   - You pick up and reposition the robot
 *   - Field-relative driving feels "off" because the gyro's zero doesn't
 *     match the actual field forward direction
 *
 * Bound to the driver's Y button with "whileTrue".
 */
public class ResetNavX extends Command {
    private final SwerveSubsystem m_drivetrain;

    /**
     * @param drivetrain  the swerve subsystem whose NavX gyro will be reset
     */
    public ResetNavX(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Resets the heading to 0 every 20ms while the button is held. */
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
