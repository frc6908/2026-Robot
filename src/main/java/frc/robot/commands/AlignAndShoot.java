package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignAndShoot extends ParallelCommandGroup {
    public AlignAndShoot(SwerveSubsystem drivetrain, ShooterMechanism shooter) {
        addCommands(
            new AlignToTag(drivetrain, () -> 0.0, () -> 0.0),
            new AutoShooter(shooter, drivetrain)
        );
    }
}
