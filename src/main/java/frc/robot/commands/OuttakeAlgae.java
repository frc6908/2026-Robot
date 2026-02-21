package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeMechanism;

/**
 * Runs the intake rollers in reverse to spit out (outtake) algae.
 *
 * Works the same as IntakeAlgae, but the motor spins in the opposite direction.
 * Bound to the operator's X button with "whileTrue" -- rollers spin while X is
 * held, and stop when X is released.
 */
public class OuttakeAlgae extends Command {
    private final AlgaeMechanism m_algaeMech;

    /**
     * Creates a new OuttakeAlgae command.
     *
     * @param algaeMech  the algae mechanism subsystem that has the intake motor
     */
    public OuttakeAlgae(AlgaeMechanism algaeMech) {
        m_algaeMech = algaeMech;
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {}

    /**
     * Spins the rollers at the outtake speed (negative = reverse direction).
     */
    @Override
    public void execute() {
        m_algaeMech.setIOSpark(AlgaeConstants.outtakeSpeed);
    }

    /** Stops the rollers when the button is released or the command is interrupted. */
    @Override
    public void end(boolean interrupted) {
        m_algaeMech.stopIOSpark();
    }

    /** Never finishes on its own -- runs until the button is released. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
