package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.ShooterMechanism;

public class Shooter extends Command {
    private final ShooterMechanism m_shooterMech;
    private final IntakeMechanism m_intakeMech;

    public Shooter(ShooterMechanism shooterMech, IntakeMechanism intakeMech) {
        m_shooterMech = shooterMech;
        m_intakeMech = intakeMech;
        addRequirements(shooterMech, intakeMech);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
       // m_shooterMech.setIOSpark(.3, -.55); One Meter
       m_shooterMech.setIOSpark(.55, -.60); //3.5 M (.55 both), 4.5 M (.55,-60)
        //ONLY USE FOR HAIL MARY LONG SHOT
       m_intakeMech.setIOSpark(IntakeConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
        m_intakeMech.stopIOSpark();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
