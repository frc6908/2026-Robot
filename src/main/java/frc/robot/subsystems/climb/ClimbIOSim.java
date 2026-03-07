package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of ClimbIO -- uses WPILib physics simulation
 * instead of real hardware.
 *
 * Simulates the climb motor as a flywheel with a single NEO motor. Good enough
 * for testing that commands activate the motor in the right direction.
 *
 * Only used in simulation. On the real robot, ClimbIOSparkMax is used instead.
 */
public class ClimbIOSim implements ClimbIO {

    /** Physics simulation of the climb motor as a simple flywheel. */
    private final FlywheelSim sim;

    /** Tracks the last speed we applied so we can log it. */
    private double appliedSpeed = 0.0;

    public ClimbIOSim() {
        // Simulate a single NEO motor driving a small flywheel (MOI = 0.001 kg*m^2)
        sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 1.0),
            DCMotor.getNEO(1)
        );
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        // Advance the physics simulation by one robot loop (20ms)
        sim.update(0.02);

        inputs.appliedSpeed = appliedSpeed;
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setSpeed(double speed) {
        appliedSpeed = speed;
        sim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        appliedSpeed = 0.0;
        sim.setInputVoltage(0.0);
    }
}
