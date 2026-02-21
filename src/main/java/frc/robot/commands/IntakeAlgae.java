package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeMechanism;

/**
 * Runs the intake rollers to suck in algae.
 *
 * This command is bound to the operator's B button with "whileTrue" --
 * meaning the rollers spin while B is held down, and stop when B is released.
 * The intake speed is defined in AlgaeConstants (75% power by default).
 *
 * WANT TO CHANGE the intake speed? → Edit AlgaeConstants.intakeSpeed in Constants.java
 * WANT TO CHANGE the button? → Edit the binding in RobotContainer.configureBindings()
 */
public class IntakeAlgae extends Command {
    private final AlgaeMechanism m_algaeMech;

    /**
     * Creates a new IntakeAlgae command.
     *
     * @param algaeMech  the algae mechanism subsystem that has the intake motor
     */
    public IntakeAlgae(AlgaeMechanism algaeMech) {
        m_algaeMech = algaeMech;
        // Declare that this command needs exclusive control of the algae mechanism
        addRequirements(algaeMech);
    }

    @Override
    public void initialize() {}

    /** Spins the intake rollers at the configured intake speed every 20ms. */
    @Override
    public void execute() {
        m_algaeMech.setIOSpark(AlgaeConstants.intakeSpeed);
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
