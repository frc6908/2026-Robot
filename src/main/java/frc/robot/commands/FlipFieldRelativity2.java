package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Enables robot-relative driving mode (disables field-relative).
 *
 * Robot-relative means when you push the joystick "forward", the robot moves
 * wherever its front is currently pointing. If the robot is turned sideways,
 * "forward" on the joystick moves sideways on the field.
 *
 * This mode doesn't need the gyroscope and can be useful in certain situations,
 * but most drivers find field-relative easier to use.
 *
 * Bound to the driver's A button with "whileTrue".
 *
 * See also: FlipFieldRelativity (which enables field-relative mode).
 */
public class FlipFieldRelativity2 extends Command {
    private final SwerveSubsystem m_drivetrain;

    /**
     * @param drivetrain  the swerve subsystem whose field relativity setting will be changed
     */
    public FlipFieldRelativity2(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Sets field-relative mode to OFF every 20ms while the button is held. */
    @Override
    public void execute() {
        m_drivetrain.setFieldRelativity(false);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
