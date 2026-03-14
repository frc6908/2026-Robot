package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperMechanism;
import frc.robot.subsystems.ShooterMechanism;

public class TrenchShootAuto extends Command {
    private final ShooterMechanism m_shooterMech;
    private final HopperMechanism m_hopperMech;
    private static final double DURATION_SECONDS = 15.0;
    private double m_startTime;

    public TrenchShootAuto(ShooterMechanism shooterMech, HopperMechanism hopperMech) {
        m_shooterMech = shooterMech;
        m_hopperMech = hopperMech;
        addRequirements(shooterMech, hopperMech);
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // 7.84m (7m + 33in) shot speeds + kicker
        m_shooterMech.setIOSpark(0.53, -0.53);
        // hopper feeds balls into shooter
        m_hopperMech.setHopperSpark(HopperConstants.intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMech.stopIOSpark();
        m_hopperMech.stopHopperSpark();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - m_startTime) >= DURATION_SECONDS;
    }
}
