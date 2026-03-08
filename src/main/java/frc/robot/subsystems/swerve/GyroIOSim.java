package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import edu.wpi.first.units.Units;

/**
 * Simulation implementation of GyroIO -- uses maple-sim's gyro simulation
 * instead of real hardware.
 *
 * maple-sim's GyroSimulation (created from COTS.ofNav2X()) provides realistic
 * yaw measurements with simulated drift, matching the behavior of a real NavX.
 *
 * Only used in simulation. On the real robot, GyroIONavX is used instead.
 */
public class GyroIOSim implements GyroIO {

    /** The maple-sim gyro simulation that provides simulated yaw values. */
    private final GyroSimulation gyroSim;

    /**
     * Creates a new GyroIOSim backed by a maple-sim gyro simulation.
     *
     * @param gyroSim the maple-sim GyroSimulation from the drivetrain sim
     */
    public GyroIOSim(GyroSimulation gyroSim) {
        this.gyroSim = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true; // Simulated gyro is always "connected"
        inputs.yawDegrees = gyroSim.getGyroReading().getDegrees();
        inputs.yawRateDegreesPerSec = gyroSim.getMeasuredAngularVelocity().in(
            Units.DegreesPerSecond);
    }

    @Override
    public void reset() {
        // maple-sim doesn't support mid-run resets, but the heading offset
        // is handled in SwerveSubsystem via pose estimator reset
    }
}
