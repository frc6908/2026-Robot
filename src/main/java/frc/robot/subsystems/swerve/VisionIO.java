package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what the vision camera hardware can do.
 *
 * The Limelight camera detects AprilTags on the field and provides:
 *   - Whether a target is visible
 *   - Horizontal offset to the target (tx)
 *   - Which AprilTag ID is visible
 *   - Distance to the target
 *   - Robot pose from the camera's perspective
 *
 * WANT TO ADD A NEW CAMERA VALUE? Add a field to VisionIOInputs and read it in
 * both VisionIOLimelight and VisionIOSim.
 */
public interface VisionIO {

    /**
     * Sensor readings from the vision camera.
     */
    @AutoLog
    class VisionIOInputs {
        /** Whether the camera currently sees a valid AprilTag target. */
        public boolean hasTarget = false;

        /** Horizontal offset to the target in degrees (negative = left, positive = right). */
        public double tx = 0.0;

        /** The AprilTag ID the camera is looking at, or -1 if none. */
        public int targetId = -1;

        /** Distance to the target in meters, or -1 if no target. */
        public double targetDistanceMeters = -1.0;

        /**
         * Robot pose from the camera's perspective (WPILib blue origin).
         * Array format: [x, y, z, roll, pitch, yaw, latency_ms]
         */
        public double[] botpose = new double[7];
    }

    /**
     * Reads the latest vision data into the inputs object.
     * Called every 20ms.
     */
    default void updateInputs(VisionIOInputs inputs) {}
}
