package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeMechanism;

/**
 * Runs the intake roller in reverse to spit out a game piece.
 *
 * This command spins the intake motor at the speed defined in IntakeConstants.outtakeSpeed
 * (negative = outward/reverse). It runs continuously while the button is held down and
 * stops the motor when the button is released.
 *
 * Bound to: Operator controller X button (whileTrue -- runs while held, stops on release).
 *
 * WANT TO CHANGE outtake speed? See IntakeConstants.outtakeSpeed in Constants.java.
 */
public class Outtake extends Command {
    private final IntakeMechanism m_intakeMech;

    public Outtake(IntakeMechanism intakeMech) {
        m_intakeMech = intakeMech;
        addRequirements(intakeMech);
    }

    @Override
    public void initialize() {}

    /** Runs every 20ms: sets the intake motor to the configured outtake speed (reverse). */
    @Override
    public void execute() {
        m_intakeMech.setIOSpark(IntakeConstants.outtakeSpeed);
    }

    /** When the command ends (button released), stop the motor. */
    @Override
    public void end(boolean interrupted) {
        m_intakeMech.stopIOSpark();
    }

    /** Never finishes on its own -- runs until the button is released. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
