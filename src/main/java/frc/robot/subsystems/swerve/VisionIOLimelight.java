package frc.robot.subsystems.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

/**
 * Hardware implementation of VisionIO -- reads from the real Limelight camera
 * via NetworkTables.
 *
 * The Limelight publishes AprilTag detection data to NetworkTables, which we
 * read here and pack into VisionIOInputs for the subsystem to use.
 *
 * Only used on the real robot. In simulation, VisionIOSim is used instead.
 */
public class VisionIOLimelight implements VisionIO {

    /** Connection to the Limelight's NetworkTable entries. */
    private final NetworkTable limelightTable;

    public VisionIOLimelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // tv = "target valid" -- 1.0 means the Limelight sees an AprilTag
        inputs.hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1.0;

        // tx = horizontal offset to the target in degrees
        inputs.tx = limelightTable.getEntry("tx").getDouble(0);

        // tid = AprilTag ID currently being tracked
        inputs.targetId = (int) limelightTable.getEntry("tid").getDouble(-1);

        // Calculate distance from 3D target pose in camera space
        double[] targetpose = limelightTable.getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[0]);
        if (targetpose.length >= 3) {
            inputs.targetDistanceMeters = Math.sqrt(
                targetpose[0] * targetpose[0]
                + targetpose[1] * targetpose[1]
                + targetpose[2] * targetpose[2]);
        } else {
            inputs.targetDistanceMeters = -1.0;
        }

        // Robot pose from Limelight's MegaTag (WPILib blue alliance origin)
        inputs.botpose = limelightTable.getEntry("botpose_wpiblue")
            .getDoubleArray(new double[7]);
    }
}
