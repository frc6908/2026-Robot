package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what a single swerve module's hardware can do.
 *
 * Each swerve module has a drive motor (moves the wheel forward/backward) and
 * a rotation motor (steers the wheel). This interface lets us read sensor data
 * and send voltage commands to either real hardware or a simulator.
 *
 * WANT TO ADD A NEW SENSOR? Add a field to SwerveModuleIOInputs and read it in
 * both SwerveModuleIOSparkMax and SwerveModuleIOSim.
 */
public interface SwerveModuleIO {

    /**
     * Sensor readings from one swerve module.
     *
     * The @AutoLog annotation tells AdvantageKit to automatically generate
     * logging code for these fields.
     */
    @AutoLog
    class SwerveModuleIOInputs {
        /** Total distance the drive wheel has traveled (in meters). */
        public double drivePositionMeters = 0.0;

        /** Current drive wheel speed (in meters per second). */
        public double driveVelocityMPS = 0.0;

        /** Voltage being applied to the drive motor. */
        public double driveAppliedVolts = 0.0;

        /** Current draw of the drive motor (in Amps). */
        public double driveCurrentAmps = 0.0;

        /** Current wheel angle (in radians), from the absolute encoder. */
        public double rotationPositionRad = 0.0;

        /** Current wheel rotation speed (in radians per second). */
        public double rotationVelocityRadPerSec = 0.0;

        /** Voltage being applied to the rotation motor. */
        public double rotationAppliedVolts = 0.0;

        /** Current draw of the rotation motor (in Amps). */
        public double rotationCurrentAmps = 0.0;
    }

    /**
     * Reads the latest sensor values from the hardware into the inputs object.
     * Called every 20ms by the module's periodic update.
     */
    default void updateInputs(SwerveModuleIOInputs inputs) {}

    /**
     * Sets the voltage to apply to the drive motor.
     *
     * @param volts voltage to apply (-12 to 12)
     */
    default void setDriveVoltage(double volts) {}

    /**
     * Sets the voltage to apply to the rotation (steering) motor.
     *
     * @param volts voltage to apply (-12 to 12)
     */
    default void setRotationVoltage(double volts) {}

    /** Stops both the drive and rotation motors immediately. */
    default void stop() {}
}
