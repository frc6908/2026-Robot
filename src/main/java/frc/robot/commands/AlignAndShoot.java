package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A composite command that aligns the robot to an AprilTag AND shoots simultaneously.
 *
 * This extends ParallelCommandGroup, which means both AlignToTag and AutoShooter run
 * at the same time. The robot auto-rotates to face the HUB while the shooter spins up
 * and fires at the correct speed for the current distance.
 *
 * Used primarily in autonomous routines (registered as the "AlignAndShoot" named command
 * with a 3-second timeout in RobotContainer).
 *
 * During autonomous, the AlignToTag command receives () -> 0.0 for both translation
 * inputs, meaning the robot only rotates (no manual driving).
 *
 * The command ends when BOTH sub-commands finish (ParallelCommandGroup behavior), or
 * when the timeout expires (if one was set).
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
