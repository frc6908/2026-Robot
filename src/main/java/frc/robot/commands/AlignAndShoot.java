package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Aims at the target AND shoots at the same time.
 *
 * This runs AlignToTag and AutoShooter together (in parallel). The robot auto-rotates
 * to face the HUB while the shooter spins up and fires at the right speed for the
 * current distance.
 *
 * Used mainly during auto mode (registered as the "AlignAndShoot" named command
 * with a 3-second timeout in RobotContainer).
 *
 * During auto, the robot only rotates (no manual driving). The command ends when
 * the timeout expires.
 */
public class AlignAndShoot extends ParallelCommandGroup {
    /**
     * Creates a new AlignAndShoot command that runs alignment and shooting in parallel.
     *
     * @param drivetrain the swerve drivetrain subsystem (for alignment)
     * @param shooter    the shooter mechanism subsystem (for shooting)
     */
    public AlignAndShoot(SwerveSubsystem drivetrain, ShooterMechanism shooter) {
        addCommands(
            // Align to the nearest AprilTag (no manual translation -- auto only)
            new AlignToTag(drivetrain, () -> 0.0, () -> 0.0),
            // Auto-shoot based on distance to the target
            new AutoShooter(shooter, drivetrain)
        );
    }
}
