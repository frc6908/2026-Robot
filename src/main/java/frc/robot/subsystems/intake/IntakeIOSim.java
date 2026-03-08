package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of IntakeIO -- uses WPILib physics simulation
 * instead of real hardware.
 *
 * This simulates the intake as a flywheel with a single NEO motor and a small
 * moment of inertia. When you set a speed, it converts the duty cycle to voltage
 * and feeds it into the physics model, which calculates realistic spin-up behavior.
 *
 * Only used in simulation. On the real robot, IntakeIOSparkMax is used instead.
 */
public class IntakeIOSim implements IntakeIO {

    /** Physics simulation of the intake as a simple flywheel. */
    private final FlywheelSim sim;

    /** Tracks the last speed we applied so we can log it. */
    private double appliedSpeed = 0.0;

    public IntakeIOSim() {
        // Simulate a single NEO motor driving a small flywheel (MOI = 0.001 kg*m^2)
        sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.001, 1.0),
            DCMotor.getNEO(1)
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Advance the physics simulation by one robot loop (20ms)
        sim.update(0.02);

        inputs.appliedSpeed = appliedSpeed;
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setSpeed(double speed) {
        appliedSpeed = speed;
        // Convert duty cycle (-1 to 1) to voltage (assuming 12V battery)
        sim.setInputVoltage(speed * 12.0);
    }

    @Override
    public void stop() {
        appliedSpeed = 0.0;
        sim.setInputVoltage(0.0);
    }
}
