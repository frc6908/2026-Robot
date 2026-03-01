package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbMechanism;

public class Climb extends Command {
    private final ClimbMechanism m_climbMech;
    public Climb(ClimbMechanism climbMech) {
        m_climbMech = climbMech;
        addRequirements(climbMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climbMech.setIOSpark(.75);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climbMech.stopIOSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
