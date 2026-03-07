package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbMechanism;

/**
 * Runs the climb motor in reverse to lower the climbing mechanism.
 *
 * This command runs the climb motor at 75% power in the "down" direction
 * (negative speed). Used to extend or reset the climbing arm/hook before
 * attempting a climb.
 *
 * Bound to: Operator controller Right Bumper (whileTrue -- lowers while held).
 *
 * WANT TO CHANGE the lowering speed? Modify the -0.75 value in execute() below.
 */
public class ClimbDown extends Command {
    private final ClimbMechanism m_climbMech;

    public ClimbDown(ClimbMechanism climbMech) {
        m_climbMech = climbMech;
        addRequirements(climbMech);
    }

    @Override
    public void initialize() {}

    /** Runs every 20ms: drives the climb motor downward at 75% power. */
    @Override
    public void execute() {
        m_climbMech.setIOSpark(-.75);
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
