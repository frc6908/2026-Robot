package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMechanism;

/**
 * Resets the algae arm encoder to zero.
 *
 * This makes the arm's current position the new "zero point." Useful for
 * recalibrating the arm after the robot has been powered on, or if the arm
 * was manually moved while disabled.
 *
 * Bound to the operator's right bumper with "whileTrue".
 */
public class ResetArmEncoder extends Command {
    private final AlgaeMechanism m_algaeMech;

    /**
     * @param algaeMech  the algae mechanism whose arm encoder will be reset
     */
    public ResetArmEncoder(AlgaeMechanism algaeMech) {
        m_algaeMech = algaeMech;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {}

    /** Resets the encoder to 0 every 20ms while the button is held. */
    @Override
    public void execute() {
        m_algaeMech.resetArmEncoder();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
