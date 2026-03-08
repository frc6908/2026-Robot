package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Enables field-relative driving mode.
 *
 * Field-relative means "up" on the joystick ALWAYS drives toward the far end of the
 * field, regardless of which way the robot is facing. This is the most intuitive mode
 * for most drivers -- it's like controlling the robot from the driver station's
 * perspective.
 *
 * This command sets fieldRelativeStatus to true while the button is held.
 *
 * Bound to: Driver controller X button (whileTrue).
 *
 * See also: FlipFieldRelativity2 (disables field-relative, enabling robot-relative mode).
 */
public class FlipFieldRelativity extends Command {
    private final SwerveSubsystem m_drivetrain;

    public FlipFieldRelativity(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    /** Sets field-relative mode to ON. */
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
