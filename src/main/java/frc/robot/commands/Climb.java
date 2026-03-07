package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbMechanism;

/**
 * Runs the climb motor to lift the robot up onto the chain.
 *
 * This command runs the climb motor at 75% power in the "up" direction.
 * It's intended for the endgame period when the robot needs to climb for
 * bonus points.
 *
 * The motor stops when the button is released, and brake mode keeps the
 * robot from sliding back down.
 *
 * Bound to: Operator controller Left Bumper (whileTrue -- climbs while held).
 *
 * WANT TO CHANGE climb speed? Modify the 0.75 value in execute() below.
 */
public class Climb extends Command {
    private final ClimbMechanism m_climbMech;

    public Climb(ClimbMechanism climbMech) {
        m_climbMech = climbMech;
        addRequirements(climbMech);
    }

    @Override
    public void initialize() {}

    /** Runs every 20ms: drives the climb motor upward at 75% power. */
    @Override
    public void execute() {
        m_climbMech.setIOSpark(.75);
    }

    /** When the command ends (button released), stop the climb motor. */
    @Override
    public void end(boolean interrupted) {
        m_climbMech.stopIOSpark();
    }

    /** Never finishes on its own -- runs until the button is released. */
    @Override
    public boolean isFinished() {
        return false;
    }

}
