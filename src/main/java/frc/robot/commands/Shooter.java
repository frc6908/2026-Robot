package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterMechanism;

/**
 * Runs the shooter at a constant speed to launch game pieces.
 *
 * This is the "dumb" shooter command -- it always runs at the same speed regardless
 * of distance to the target. Both flywheel motors spin at ShooterConstants.shooterSpeed
 * and the kicker feeds the game piece into the flywheel.
 *
 * For distance-based auto-speed shooting, use AutoShooter instead.
 *
 * Bound to: Driver controller B button (whileTrue -- runs while held, stops on release).
 *
 * WANT TO CHANGE the constant shooting speed? See ShooterConstants.shooterSpeed in Constants.java.
 */
public class Shooter extends Command {
    private final ShooterMechanism m_shooterMech;

    public Shooter(ShooterMechanism shooterMech) {
        m_shooterMech = shooterMech;
        addRequirements(shooterMech);
    }

    @Override
    public void initialize() {}

    /** Runs every 20ms: sets both shooter motors to the configured constant speed. */
    @Override
    public void execute() {
        m_shooterMech.setIOSpark(ShooterConstants.shooterSpeed, ShooterConstants.shooterSpeed);
    }

    /** When the command ends (button released), stop all shooter motors. */
    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
    }

    /** Never finishes on its own -- runs until the button is released. */
    @Override
    public boolean isFinished() {
        return false;
    }

}
