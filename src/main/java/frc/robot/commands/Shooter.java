package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperMechanism;
import frc.robot.subsystems.ShooterMechanism;

public class Shooter extends Command {
    private final ShooterMechanism m_shooterMech;
    private final HopperMechanism m_hopperMech;

    public Shooter(ShooterMechanism shooterMech, HopperMechanism hopperMech) {
        m_shooterMech = shooterMech;
        m_hopperMech = hopperMech;
        addRequirements(shooterMech);

        // Initialize sliders with current defaults
        SmartDashboard.putNumber("Shooter Speed1", 0.57);
        SmartDashboard.putNumber("Shooter Speed2", -0.57);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
       double speed1 = SmartDashboard.getNumber("Shooter Speed1", 0.65);
       double speed2 = SmartDashboard.getNumber("Shooter Speed2", -0.60);
       m_shooterMech.setIOSpark(speed1, speed2);
       m_hopperMech.setHopperSpark(HopperConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
        m_hopperMech.stopHopperSpark();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
