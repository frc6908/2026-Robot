package frc.robot.subsystems.swerve;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.units.Units;
import frc.robot.Constants.DrivetrainConstants;

/**
 * Simulation implementation of SwerveModuleIO -- uses maple-sim's physics engine
 * instead of real hardware.
 *
 * Each simulated swerve module is driven by a SwerveModuleSimulation from maple-sim,
 * which provides realistic motor physics through the dyn4j engine. We use
 * GenericMotorController to send voltage commands to the simulated motors.
 *
 * Only used in simulation. On the real robot, SwerveModuleIOSparkMax is used instead.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {

    /** The maple-sim module simulation that handles all physics. */
    private final SwerveModuleSimulation moduleSim;

    /** Controller for sending voltage to the simulated drive motor. */
    private final SimulatedMotorController.GenericMotorController driveController;

    /** Controller for sending voltage to the simulated steer motor. */
    private final SimulatedMotorController.GenericMotorController steerController;

    /** Wheel radius in meters, used to convert angular to linear measurements. */
    private static final double WHEEL_RADIUS_METERS = DrivetrainConstants.wheelDiameter / 2.0;

    /**
     * Creates a new SwerveModuleIOSim backed by a maple-sim module simulation.
     *
     * @param moduleSim the maple-sim SwerveModuleSimulation for this corner
     */
    public SwerveModuleIOSim(SwerveModuleSimulation moduleSim) {
        this.moduleSim = moduleSim;
        this.driveController = moduleSim.useGenericMotorControllerForDrive();
        this.steerController = moduleSim.useGenericControllerForSteer();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Convert angular drive measurements to linear (meters) using wheel radius
        double drivePositionRad = moduleSim.getDriveWheelFinalPosition().in(Units.Radians);
        inputs.drivePositionMeters = drivePositionRad * WHEEL_RADIUS_METERS;

        double driveSpeedRadPerSec = moduleSim.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond);
        inputs.driveVelocityMPS = driveSpeedRadPerSec * WHEEL_RADIUS_METERS;

        inputs.driveAppliedVolts = driveController.getAppliedVoltage().in(Units.Volts);
        inputs.driveCurrentAmps = moduleSim.getDriveMotorStatorCurrent().in(Units.Amps);

        inputs.rotationPositionRad = moduleSim.getSteerAbsoluteFacing().getRadians();
        inputs.rotationVelocityRadPerSec = moduleSim.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
        inputs.rotationAppliedVolts = steerController.getAppliedVoltage().in(Units.Volts);
        inputs.rotationCurrentAmps = moduleSim.getSteerMotorStatorCurrent().in(Units.Amps);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveController.requestVoltage(Units.Volts.of(volts));
    }

    @Override
    public void setRotationVoltage(double volts) {
        steerController.requestVoltage(Units.Volts.of(volts));
    }

    @Override
    public void stop() {
        driveController.requestVoltage(Units.Volts.of(0));
        steerController.requestVoltage(Units.Volts.of(0));
    }
}
