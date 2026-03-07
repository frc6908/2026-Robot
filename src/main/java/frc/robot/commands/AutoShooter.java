package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Set;

/**
 * Distance-based auto-speed shooter -- automatically adjusts flywheel speed based on
 * how far the robot is from the target.
 *
 * Unlike the basic Shooter command (which always runs at the same speed), this one
 * uses the Limelight to:
 *   1. Check which AprilTag it sees and make sure it's one of our alliance's HUB tags
 *   2. Measure how far away the target is (in meters)
 *   3. Pick the right shooter speed from a lookup table
 *      (see ShooterConstants.kDistanceToSpeedMap in Constants.java)
 *
 * The lookup table figures out the right speed for any distance, even if it's between
 * two entries. For example, if we've calibrated for 1.0m and 2.5m, the table will
 * automatically pick a speed in between for 1.75m.
 *
 * If no valid target is visible, the shooter does NOT fire (it waits for a target).
 * This prevents wasting game pieces on blind shots.
 *
 * Bound to: Driver controller Left Bumper (whileTrue -- runs while held).
 *
 * WANT TO TUNE? Update the distance-to-speed lookup table entries in
 * ShooterConstants.kDistanceToSpeedMap. Measure shooting accuracy at known distances
 * and adjust accordingly.
 */
public class AutoShooter extends Command {
    private final ShooterMechanism m_shooter;
    private final SwerveSubsystem m_drive;

    /** AprilTag IDs on the red alliance HUB. We check these to make sure we're aiming at our goal. */
    private final Set<Integer> redHubTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    /** AprilTag IDs on the blue alliance HUB. */
    private final Set<Integer> blueHubTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    public AutoShooter(ShooterMechanism shooter, SwerveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter);
    }

    /**
     * Runs every 20ms: checks for a valid target, calculates distance, looks up
     * the appropriate speed, and sets the shooter motors.
     */
    @Override
    public void execute() {
        // 1. Determine which alliance we're on and which tags are valid targets
        var alliance = DriverStation.getAlliance();
        Set<Integer> targetTags = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? redHubTags : blueHubTags;

        // 2. Check if the Limelight sees a valid HUB tag
        double closestDistance = -1.0;

        if (m_drive.getLimelightHasTarget()) {
            int tid = m_drive.getLimelightTid();

            // Only shoot if we're looking at one of our alliance's HUB tags
            if (targetTags.contains(tid)) {
                double distance = m_drive.getLimelightTargetDistanceMeters();
                if (distance > 0) {
                    closestDistance = distance;
                }
            }
        }

        // 3. Set motor speed based on distance (or do nothing if no valid target)
        if (closestDistance > 0) {
            // The lookup table picks the right speed based on how far away we are
            double speed = ShooterConstants.kDistanceToSpeedMap.get(closestDistance);

            m_shooter.setIOSpark(speed, speed);

            // Log values for debugging/tuning on SmartDashboard
            SmartDashboard.putNumber("AutoShooter/TargetDistance", closestDistance);
            SmartDashboard.putNumber("AutoShooter/SetSpeed", speed);
            SmartDashboard.putBoolean("AutoShooter/HasTarget", true);
        } else {
            SmartDashboard.putBoolean("AutoShooter/HasTarget", false);
        }
    }

    /** When the command ends, stop all shooter motors. */
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopIOSpark();
    }
}
