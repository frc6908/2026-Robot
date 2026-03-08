package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of ShooterIO -- uses WPILib physics simulation
 * instead of real hardware.
 *
 * Simulates the two shooter flywheel motors as separate flywheels with realistic
 * spin-up behavior. The kicker uses a smaller flywheel simulation.
 *
 * Only used in simulation. On the real robot, ShooterIOSparkMax is used instead.
 */
public class ShooterIOSim implements ShooterIO {

    /** Physics simulation of shooter motor 1 (flywheel with MOI ~0.004 kg*m^2). */
    private final FlywheelSim shooterSim1;

    /** Physics simulation of shooter motor 2. */
    private final FlywheelSim shooterSim2;

    /** Physics simulation of the kicker motor. */
    private final FlywheelSim kickerSim;

    private double shooter1AppliedSpeed = 0.0;
    private double shooter2AppliedSpeed = 0.0;
    private double kickerAppliedSpeed = 0.0;

    public ShooterIOSim() {
        // Shooter flywheels have a larger MOI for realistic spin-up time
        shooterSim1 = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.004, 1.0),
            DCMotor.getNEO(1)
        );
        shooterSim2 = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.004, 1.0),
            DCMotor.getNEO(1)
        );
        kickerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 1.0),
            DCMotor.getNEO(1)
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Advance physics by one robot loop (20ms)
        shooterSim1.update(0.02);
        shooterSim2.update(0.02);
        kickerSim.update(0.02);

        inputs.shooter1AppliedSpeed = shooter1AppliedSpeed;
        inputs.shooter1VelocityRadPerSec = shooterSim1.getAngularVelocityRadPerSec();
        inputs.shooter1CurrentAmps = shooterSim1.getCurrentDrawAmps();

        inputs.shooter2AppliedSpeed = shooter2AppliedSpeed;
        inputs.shooter2VelocityRadPerSec = shooterSim2.getAngularVelocityRadPerSec();
        inputs.shooter2CurrentAmps = shooterSim2.getCurrentDrawAmps();

        inputs.kickerAppliedSpeed = kickerAppliedSpeed;
        inputs.kickerVelocityRadPerSec = kickerSim.getAngularVelocityRadPerSec();
        inputs.kickerCurrentAmps = kickerSim.getCurrentDrawAmps();
    }

    @Override
    public void setShooterSpeed(double speed1, double speed2) {
        shooter1AppliedSpeed = speed1;
        shooter2AppliedSpeed = speed2;
        shooterSim1.setInputVoltage(speed1 * 12.0);
        shooterSim2.setInputVoltage(speed2 * 12.0);
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerAppliedSpeed = speed;
        kickerSim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        shooter1AppliedSpeed = 0.0;
        shooter2AppliedSpeed = 0.0;
        kickerAppliedSpeed = 0.0;
        shooterSim1.setInputVoltage(0.0);
        shooterSim2.setInputVoltage(0.0);
        kickerSim.setInputVoltage(0.0);
    }
}
