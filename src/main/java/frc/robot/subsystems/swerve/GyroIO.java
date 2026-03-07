package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what the gyroscope hardware can do.
 *
 * The gyroscope (NavX) measures which direction the robot is facing and how fast
 * it's spinning. This data is critical for field-relative driving and odometry.
 *
 * WANT TO ADD A NEW SENSOR READING? Add a field to GyroIOInputs and read it in
 * both GyroIONavX and GyroIOSim.
 */
public interface GyroIO {

    /**
     * Sensor readings from the gyroscope.
     */
    @AutoLog
    class GyroIOInputs {
        /** Whether the gyro is currently connected and sending data. */
        public boolean connected = false;

        /** Current yaw (heading) of the robot in degrees. */
        public double yawDegrees = 0.0;

        /** How fast the robot is spinning in degrees per second. */
        public double yawRateDegreesPerSec = 0.0;
    }

    /**
     * Reads the latest gyro data into the inputs object.
     * Called every 20ms.
     */
    default void updateInputs(GyroIOInputs inputs) {}

    /** Resets the gyro heading to zero. Current direction becomes "forward". */
    default void reset() {}
}
