package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a single swerve module -- one corner of the swerve drivetrain.
 *
 * Each swerve module has a drive motor (spins the wheel) and a rotation motor
 * (steers the wheel). This class uses PID controllers to hit target speeds and
 * angles, reading sensor data from a SwerveModuleIO interface.
 *
 * The IO pattern means this same code works with real hardware (SwerveModuleIOSparkMax)
 * or simulated physics (SwerveModuleIOSim). The PID controllers stay here in the
 * module -- not in the IO layer -- because PID logic is the same regardless of
 * whether we're on real hardware or in sim.
 *
 * WANT TO CHANGE steering behavior? Adjust kPRotation, kDRotation in DrivetrainConstants.
 * WANT TO CHANGE drive behavior? Adjust kPDrive in DrivetrainConstants.
 */
public class SwerveModule {

    /** The IO layer -- could be real hardware or simulation. */
    private final SwerveModuleIO io;

    /** Logged sensor readings, updated every periodic call. */
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    /** Name of this module for logging (e.g., "FL", "FR", "BL", "BR"). */
    private final String name;

    /** PID controller for steering the wheel to the target angle. */
    private final PIDController rotationPIDController;

    /** PID controller for driving the wheel at the target speed. */
    private final PIDController drivePIDController;

    /**
     * Creates a new swerve module with the given IO and configuration.
     *
     * @param io   the hardware interface (real or sim)
     * @param name the module name for logging (e.g., "FL")
     * @param rotationP the P-gain for the rotation PID controller
     */
    public SwerveModule(SwerveModuleIO io, String name, double rotationP) {
        this.io = io;
        this.name = name;

        // Drive PID: controls wheel speed to match the target velocity
        drivePIDController = new PIDController(
            DrivetrainConstants.kPDrive,
            DrivetrainConstants.kIDrive,
            DrivetrainConstants.kDDrive);

        // Rotation PID: controls wheel angle to match the target direction
        rotationPIDController = new PIDController(
            rotationP,
            DrivetrainConstants.kIRotation,
            DrivetrainConstants.kDRotation);
        rotationPIDController.setTolerance(DrivetrainConstants.kToleranceRotation);

        // Tell the PID that angles wrap around in a circle (-180 and +180 are the same).
        // Without this, the wheel might spin the long way around.
        rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Updates sensor readings from the IO layer. Should be called every 20ms.
     */
    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/" + name, inputs);
    }

    /**
     * Sets the desired state (speed + angle) for this swerve module.
     *
     * If the desired speed is essentially zero (< 0.0001 m/s), the module stops
     * entirely. This prevents the wheels from snapping to an angle when the driver
     * isn't trying to move.
     *
     * @param state the desired SwerveModuleState (speed in m/s, angle in radians)
     */
    public void setState(SwerveModuleState state) {
        // Dead zone: if speed is basically zero, just stop
        if (Math.abs(state.speedMetersPerSecond) < 0.0001) {
            stop();
            return;
        }

        // Use PID to calculate the rotation voltage needed to reach the target angle
        double rotationOutput = rotationPIDController.calculate(
            inputs.rotationPositionRad, state.angle.getRadians());
        io.setRotationVoltage(rotationOutput * 12.0);

        // Use PID to calculate the drive voltage needed to reach the target speed
        double driveOutput = drivePIDController.calculate(
            inputs.driveVelocityMPS, state.speedMetersPerSecond);
        io.setDriveVoltage(driveOutput * 12.0);
    }

    /** Returns the current state of this module (velocity + angle). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityMPS,
            new Rotation2d(inputs.rotationPositionRad));
    }

    /** Returns the current position of this module (distance + angle). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters,
            new Rotation2d(inputs.rotationPositionRad));
    }

    /** Returns the current drive velocity in meters per second. */
    public double getDriveVelocity() {
        return inputs.driveVelocityMPS;
    }

    /** Returns the current rotation position in radians. */
    public double getRotationPosition() {
        return inputs.rotationPositionRad;
    }

    /** Returns the drive position in meters (total distance traveled). */
    public double getDrivePosition() {
        return inputs.drivePositionMeters;
    }

    /** Returns the current wheel angle in radians (from absolute encoder). */
    public double getCANCoderRad() {
        return inputs.rotationPositionRad;
    }

    /** Stops both drive and rotation motors. */
    public void stop() {
        io.stop();
    }
}
