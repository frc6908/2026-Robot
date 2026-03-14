package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HopperMechanism;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Set;

public class AutoShooter extends Command {
    private final ShooterMechanism m_shooter;
    private final SwerveSubsystem m_drive;
    private final HopperMechanism m_hopper;

    private final Set<Integer> redHubTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private final Set<Integer> blueHubTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    public AutoShooter(ShooterMechanism shooter, SwerveSubsystem drive, HopperMechanism hopper) {
        m_shooter = shooter;
        m_drive = drive;
        m_hopper = hopper;
        addRequirements(m_shooter, m_hopper);
    }

    @Override
    public void execute() {
        // 1. Determine Alliance & Valid Tags
        var alliance = DriverStation.getAlliance();
        Set<Integer> targetTags = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? redHubTags : blueHubTags;

        // 2. Get Camera Data (Limelight)
        double closestDistance = -1.0;

        if (m_drive.getLimelightHasTarget()) {
            int tid = m_drive.getLimelightTid();

            // Check if the primary target is one of our alliance's hub tags
            if (targetTags.contains(tid)) {
                double distance = m_drive.getLimelightTargetDistanceMeters();
                if (distance > 0) {
                    closestDistance = distance;
                }
            }
        }

        // 3. Set Motor Speed
        if (closestDistance > 0) {
            // Clamp to tree map range (1m - 12m)
            double clampedDistance = Math.max(1.0, Math.min(12.0, closestDistance));
            double speed1 = ShooterConstants.kDistanceToSpeed1Map.get(clampedDistance);
            double speed2 = ShooterConstants.kDistanceToSpeed2Map.get(clampedDistance);

            m_shooter.setIOSpark(speed1, speed2);
            m_hopper.setHopperSpark(HopperConstants.intakeSpeed);

            // Log for debugging/tuning
            SmartDashboard.putNumber("AutoShooter/TargetDistance", closestDistance);
            SmartDashboard.putNumber("AutoShooter/SetSpeed1", speed1);
            SmartDashboard.putNumber("AutoShooter/SetSpeed2", speed2);
            SmartDashboard.putBoolean("AutoShooter/HasTarget", true);
        } else {
            SmartDashboard.putBoolean("AutoShooter/HasTarget", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopIOSpark();
        m_hopper.stopHopperSpark();
    }
}
