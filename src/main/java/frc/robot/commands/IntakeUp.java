package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BarConstants;
import frc.robot.subsystems.BarMechanism;

public class IntakeUp extends Command {
    private final BarMechanism m_barUpMech;
    public IntakeUp(BarMechanism barMech) {
        m_barUpMech = barMech;
        addRequirements(m_barUpMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_barUpMech.setBarSpark(BarConstants.barUpSpeed);
        m_barUpMech.setBarSpark2(BarConstants.barUpSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_barUpMech.setBarSpark(0.0);
        m_barUpMech.setBarSpark2(0.0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
