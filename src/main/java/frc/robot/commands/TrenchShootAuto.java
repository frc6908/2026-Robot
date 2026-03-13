package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.ShooterMechanism;

public class TrenchShootAuto extends Command {
    private final ShooterMechanism m_shooterMech;
    private final IntakeMechanism m_intakeMech;
    private static final double DURATION_SECONDS = 15.0;
    private double m_startTime;

    public TrenchShootAuto(ShooterMechanism shooterMech, IntakeMechanism intakeMech) {
        m_shooterMech = shooterMech;
        m_intakeMech = intakeMech;
        addRequirements(shooterMech, intakeMech);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // 4.5m shot speeds + kicker
        m_shooterMech.setIOSpark(0.55, -0.60);
        // intake feeds balls into shooter
        m_intakeMech.setIOSpark(IntakeConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
        m_intakeMech.stopIOSpark();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - m_startTime) >= DURATION_SECONDS;
    }
}
