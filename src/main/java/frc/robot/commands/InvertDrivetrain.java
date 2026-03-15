package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class InvertDrivetrain extends Command {
    private final SwerveSubsystem m_drivetrain;

    public InvertDrivetrain(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.setInvertedControls(!SwerveSubsystem.invertedControls);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
