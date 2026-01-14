package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Set;

public class AutoShooter extends Command {
    private final ShooterMechanism m_shooter;
    private final SwerveSubsystem m_drive;

    private final Set<Integer> redHubTags = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private final Set<Integer> blueHubTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    public AutoShooter(ShooterMechanism shooter, SwerveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        // 1. Determine Alliance & Valid Tags
        var alliance = DriverStation.getAlliance();
        Set<Integer> targetTags = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) 
            ? redHubTags : blueHubTags;

        // 2. Get Camera Data
        PhotonPipelineResult result = m_drive.getCameraResult();
        double closestDistance = -1.0;

        if (result.hasTargets()) {
            // Loop through all tags the camera sees
            for (PhotonTrackedTarget target : result.getTargets()) {
                int id = target.getFiducialId();

                // If this tag is part of our Hub
                if (targetTags.contains(id)) {
                    // Calculate distance using the 3D transform (hypotenuse of X and Y)
                    // The 'BestCameraToTarget' transform is the distance from the lens to the tag
                    double distance = target.getBestCameraToTarget().getTranslation().getNorm();

                    // Keep the closest one
                    if (closestDistance == -1.0 || distance < closestDistance) {
                        closestDistance = distance;
                    }
                }
            }
        }

        // 3. Set Motor Speed
        if (closestDistance > 0) {
            // Look up the speed in our table
            double speed = ShooterConstants.kDistanceToSpeedMap.get(closestDistance);
            
            m_shooter.setIOSpark(speed, speed);
            
            // Log for debugging/tuning
            SmartDashboard.putNumber("AutoShooter/TargetDistance", closestDistance);
            SmartDashboard.putNumber("AutoShooter/SetSpeed", speed);
            SmartDashboard.putBoolean("AutoShooter/HasTarget", true);
        } else {
            SmartDashboard.putBoolean("AutoShooter/HasTarget", false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopIOSpark();
    }
}