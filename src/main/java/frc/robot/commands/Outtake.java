package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperMechanism;

public class Outtake extends Command {
    private final HopperMechanism m_hopperMech;

    public Outtake(HopperMechanism hopperMech) {
        m_hopperMech = hopperMech;
        addRequirements(hopperMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_hopperMech.setHopperSpark(HopperConstants.outtakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopperMech.stopHopperSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
