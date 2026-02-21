package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Enables field-relative driving mode.
 *
 * Field-relative means when you push the joystick "forward", the robot moves
 * toward the far end of the field -- no matter which way the robot is currently
 * facing. This is the more intuitive mode for most drivers because it feels
 * like controlling a character in a video game with a top-down camera.
 *
 * This requires the NavX gyroscope to know which way the robot is facing.
 *
 * Bound to the driver's X button with "whileTrue". While X is held, field-relative
 * mode stays on. (In practice, it sets a flag that persists even after release.)
 *
 * See also: FlipFieldRelativity2 (which disables field-relative mode).
 */
public class FlipFieldRelativity extends Command {
    private final SwerveSubsystem m_drivetrain;

    /**
     * @param drivetrain  the swerve subsystem whose field relativity setting will be changed
     */
    public FlipFieldRelativity(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Sets field-relative mode to ON every 20ms while the button is held. */
    @Override
    public void execute() {
        m_drivetrain.setFieldRelativity(true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
