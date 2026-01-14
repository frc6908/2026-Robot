package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterMechanism;

public class Shooter extends Command {
    private final ShooterMechanism m_shooterMech;
    public Shooter(ShooterMechanism shooterMech) {
        m_shooterMech = shooterMech;
        addRequirements(shooterMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooterMech.setIOSpark(ShooterConstants.shooterSpeed1, ShooterConstants.shooterSpeed2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
