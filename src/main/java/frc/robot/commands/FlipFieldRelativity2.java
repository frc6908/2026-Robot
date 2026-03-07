package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Enables robot-relative driving mode (disables field-relative).
 *
 * Robot-relative means "up" on the joystick drives wherever the FRONT of the robot
 * is currently pointing. If the robot is facing sideways, "up" moves sideways.
 * This can be useful for precise maneuvers but is less intuitive for most drivers.
 *
 * This command sets fieldRelativeStatus to false while the button is held.
 *
 * Bound to: Driver controller A button (whileTrue).
 *
 * See also: FlipFieldRelativity (enables field-relative mode).
 */
public class FlipFieldRelativity2 extends Command {
    private final SwerveSubsystem m_drivetrain;

    public FlipFieldRelativity2(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Sets field-relative mode to OFF (robot-relative). */
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
