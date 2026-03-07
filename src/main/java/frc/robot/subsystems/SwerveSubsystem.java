package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * The swerve drivetrain subsystem -- the heart of robot movement.
 *
 * A swerve drive has 4 independently steerable wheels. Each wheel can point in any direction
 * AND spin at any speed, which means the robot can drive in any direction while simultaneously
 * rotating. Imagine a shopping cart where every wheel can be controlled independently -- that's
 * swerve drive.
 *
 * This subsystem manages:
 *   - Four SwerveModules (one per corner: FL, FR, BL, BR)
 *   - A NavX gyroscope (measures which direction the robot is facing)
 *   - A SwerveDrivePoseEstimator (fuses encoder + gyro + Limelight vision data to know
 *     WHERE the robot is on the field)
 *   - Limelight vision camera integration (reads AprilTags for distance/angle/pose data)
 *   - PathPlanner AutoBuilder integration (allows autonomous routines to drive the robot)
 *
 * The key concept: every 20ms, the periodic() method updates the robot's estimated position
 * using wheel encoder readings AND vision measurements from the Limelight. This "sensor fusion"
 * gives us a more accurate position than either source alone.
 *
 * WANT TO CHANGE how the robot drives? Look at the drive() method.
 * WANT TO CHANGE vision behavior? Look at updateVisionPose().
 * WANT TO CHANGE PathPlanner PID? Look at the AutoBuilder.configure() call in the constructor.
 */
public class SwerveSubsystem extends SubsystemBase{

    /**
     * Whether driving is field-relative (true) or robot-relative (false).
     * Field-relative: pushing the stick "up" always drives toward the far end of the field.
     * Robot-relative: pushing the stick "up" drives wherever the robot's front is pointing.
     * Static so it can be accessed from commands like FlipFieldRelativity.
     */
    public static boolean fieldRelativeStatus = true;

    // ========================
    // SWERVE MODULES
    // ========================
    // One module per corner. Each has a drive motor, rotation motor, and CANcoder.
    // The order (FL, FR, BL, BR) must match the order in SwerveDriveKinematics.

    private final SwerveModule frontLeft = new SwerveModule(
        DrivetrainConstants.kFLDrive,
        DrivetrainConstants.kFLRotate,
        DrivetrainConstants.kFLCanCoder,
        DrivetrainConstants.kFLOffsetRad,
        DrivetrainConstants.fLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule frontRight = new SwerveModule(
        DrivetrainConstants.kFRDrive,
        DrivetrainConstants.kFRRotate,
        DrivetrainConstants.kFRCanCoder,
        DrivetrainConstants.kFROffsetRad,
        DrivetrainConstants.fRIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backLeft = new SwerveModule(
        DrivetrainConstants.kBLDrive,
        DrivetrainConstants.kBLRotate,
        DrivetrainConstants.kBLCanCoder,
        DrivetrainConstants.kBLOffsetRad,
        DrivetrainConstants.bLIsInverted,
        DrivetrainConstants.kPRotation
    );
    private final SwerveModule backRight = new SwerveModule(
        DrivetrainConstants.kBRDrive,
        DrivetrainConstants.kBRRotate,
        DrivetrainConstants.kBRCanCoder,
        DrivetrainConstants.kBROffsetRad,
        DrivetrainConstants.bRIsInverted,
        DrivetrainConstants.kPRotation
    );

    // ========================
    // SENSORS
    // ========================

    /** NavX gyroscope -- measures the robot's heading (which direction it's facing). */
    private final AHRS navX;

    /**
     * Pose Estimator -- combines wheel encoders, gyro, AND vision measurements to
     * estimate the robot's position on the field. More accurate than odometry alone
     * because vision corrections prevent drift over time.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /** NetworkTable for the Limelight camera. All camera data is read/written through this. */
    private final NetworkTable limelightTable;



    /**
     * Constructor -- sets up all hardware and configures PathPlanner.
     *
     * The NavX reset is done in a separate thread with a 1-second delay because the
     * NavX gyro takes about a second to boot up after power-on. If we tried to reset
     * it immediately, it might not be ready yet.
     */
    public SwerveSubsystem(){
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // Delay NavX reset by 1 second to allow it to finish booting up.
        // The NavX needs time to calibrate its internal sensors after power-on.
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX.reset();
            }
            catch(Exception e){
                e.printStackTrace();
            }
        }).start();

        // Initialize rotation offsets so the relative encoders match the CANcoder positions.
        // CANcoders are "absolute" (they always know the real angle), while relative encoders
        // only track changes from their starting position. This syncs them up.
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        // Reset drive encoders to zero. We start measuring distance from here.
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        // --- Vision Initialization (Limelight) ---
        // Connect to the Limelight's NetworkTable so we can read camera data.
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);

        // Add the Limelight camera stream to the Shuffleboard "Driver" tab
        // so the drive team can see what the camera sees.
        HttpCamera cameraStream = new HttpCamera("LimelightStream",
            "http://" + VisionConstants.kLimelightName + ".local:5800/stream.mjpg");

        Shuffleboard.getTab("Driver")
            .add("Limelight", cameraStream)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(4, 3);

        // --- Pose Estimator Initialization ---
        // Start with a known position (0,0) facing forward (0 degrees).
        // This gets updated every 20ms with encoder + vision data.
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SwerveDriveKinematics,
            new Rotation2d(), // Initial gyro angle
            getModulePositions(),
            new Pose2d() // Initial position (0, 0, 0 degrees)
        );

        // --- PathPlanner Configuration ---
        // Load the robot configuration from PathPlanner's GUI settings file.
        // This includes robot mass, moment of inertia, and other physical properties
        // that PathPlanner needs to generate accurate paths.
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
        }

        // Configure PathPlanner's AutoBuilder so it can drive the robot during autonomous.
        // We tell it how to get the robot's pose, how to reset the pose, how to get
        // current speeds, and how to drive the robot.
        AutoBuilder.configure(
            this::getPose, // How to get the robot's current position
            this::resetOdometry, // How to reset the robot's position (used at start of auto)
            this::getRobotChassisSpeeds, // How to get current robot speeds (MUST be robot-relative)
            (speeds, feedforwards) -> driveRobotRelative(speeds), // How to drive the robot
            new PPHolonomicDriveController( // PID controller for path following
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID (driving to position)
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (turning to angle)
            ),
            config, // Robot physical configuration
            () -> {
              // Should paths be mirrored for the red alliance?
              // Paths are designed from the blue side. If we're on red, flip them.
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // This subsystem -- PathPlanner will "require" it during auto
        );
    }


    /**
     * Stops all four swerve modules (both drive and rotation motors).
     * Called when we want the robot to come to a complete stop.
     */
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Returns the robot's estimated position on the field.
     * This fuses encoder data with vision measurements for maximum accuracy.
     *
     * @return the robot's estimated Pose2d (x, y, heading) on the field
     */
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the robot's current chassis speeds (how fast it's moving and rotating).
     * Uses the wheel module states to calculate overall robot motion.
     * This is in ROBOT-RELATIVE coordinates (not field-relative).
     *
     * @return ChassisSpeeds representing the robot's current motion
     */
    public ChassisSpeeds getRobotChassisSpeeds(){
        return DrivetrainConstants.SwerveDriveKinematics.toChassisSpeeds(getStates());
    }

    /**
     * Drives the robot using robot-relative ChassisSpeeds.
     * Used by PathPlanner during autonomous to control the robot.
     * Desaturates wheel speeds to ensure no module exceeds the maximum velocity.
     *
     * @param speeds the desired robot-relative chassis speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        setModuleStates(moduleStates);
    }

    /**
     * Returns the robot's current heading (which direction it's facing) from the NavX gyro.
     * Negated because the NavX reports clockwise as positive, but WPILib uses
     * counter-clockwise as positive (math convention).
     *
     * @return the robot's heading as a Rotation2d
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    /**
     * Returns the raw NavX gyroscope object. Useful for advanced operations.
     *
     * @return the AHRS (NavX) gyroscope
     */
    public AHRS getNavX(){
        return navX;
    }

    /**
     * Resets the NavX heading to zero. After calling this, the robot's current facing
     * direction becomes the new "forward" (0 degrees). Useful when the robot's heading
     * has drifted or when you want to re-zero at the start of a match.
     */
    public void resetHeading(){
        navX.reset();
    }

    /**
     * Resets the pose estimator to a specific position on the field.
     * Used at the start of autonomous to tell the robot where it's starting.
     *
     * @param pose the position to reset to (x, y, heading)
     */
    public void resetOdometry(Pose2d pose){
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /**
     * Sends desired states (speed + angle) to each swerve module.
     * Also publishes the set angles to SmartDashboard for debugging.
     *
     * @param desiredStates array of 4 SwerveModuleStates [FL, FR, BL, BR]
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);

        // Log the commanded angles for debugging
        SmartDashboard.putNumber("FL Set Position", desiredStates[0].angle.getRadians());
        SmartDashboard.putNumber("FR Set Position", desiredStates[1].angle.getRadians());
        SmartDashboard.putNumber("BL Set Position", desiredStates[2].angle.getRadians());
        SmartDashboard.putNumber("BR Set Position", desiredStates[3].angle.getRadians());
    }

    /**
     * Returns the current position of each swerve module (distance traveled + angle).
     * Used by the pose estimator to track robot movement.
     *
     * @return array of 4 SwerveModulePositions [FL, FR, BL, BR]
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad())),
        };
        return positions;
    }

    /**
     * Returns the current state of each swerve module (velocity + angle).
     * Used for kinematics calculations and logging.
     *
     * @return array of 4 SwerveModuleStates [FL, FR, BL, BR]
     */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    /**
     * Sets whether driving should be field-relative or robot-relative.
     *
     * @param relativity true for field-relative, false for robot-relative
     */
    public void setFieldRelativity(boolean relativity){
        fieldRelativeStatus = relativity;
    }

    /**
     * The main drive method. Takes desired forward, strafe, and rotation speeds and
     * converts them into individual swerve module states.
     *
     * If field-relative, the speeds are relative to the field (pushing "up" always goes
     * toward the far wall). If robot-relative, the speeds are relative to the robot's
     * current heading (pushing "up" goes wherever the front of the robot is pointing).
     *
     * The discretize() call compensates for the fact that the robot is moving while we
     * calculate -- it prevents the "skew" that would otherwise happen during high-speed
     * turns.
     *
     * @param forward  forward/backward speed in m/s (positive = forward)
     * @param strafe   left/right speed in m/s (positive = left)
     * @param rotation rotational speed in rad/s (positive = counter-clockwise)
     * @param isFieldRelative whether to drive field-relative (true) or robot-relative (false)
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative){
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);

        // Discretize compensates for the robot moving during the 20ms control loop period.
        // Without it, the robot would "skew" during combined translation + rotation movements.
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Log chassis speeds for AdvantageKit replay/analysis
        Logger.recordOutput("ChassisSpeeds", speeds);

        // Convert chassis speeds into individual module states and cap them
        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);
        setModuleStates(states);

        // Log commanded speeds for debugging
        SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
    }

    /**
     * Called every 20ms by the CommandScheduler. Updates the pose estimator with
     * fresh encoder + gyro data and vision measurements, then publishes debug info.
     */
    @Override
    public void periodic(){
        super.periodic();

        // Update pose estimator with wheel encoder + gyro data (always accurate, but drifts over time)
        poseEstimator.update(getHeading(), getModulePositions());

        // Update pose estimator with Limelight vision data (corrects for drift)
        updateVisionPose();

        // Log data for AdvantageKit
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());

        // Publish debug telemetry
        execute();
    }

    /**
     * Updates the SwerveDrivePoseEstimator with vision measurements from the Limelight.
     *
     * The Limelight provides "botpose_wpiblue" -- the robot's estimated position on the
     * field using the WPILib blue-origin coordinate system. The array format is:
     * [x, y, z, roll, pitch, yaw, total_latency_ms]
     *
     * The latency compensation is important: the Limelight image was captured some
     * milliseconds ago, so we subtract the latency from the current time to tell the
     * pose estimator WHEN the measurement was taken, not when we received it.
     *
     * This method silently returns (does nothing) if:
     *   - No target is visible (tv != 1)
     *   - The pose data is incomplete or contains invalid values (NaN/Infinity)
     */
    private void updateVisionPose() {
        // tv = "target valid" -- 1.0 means the Limelight sees an AprilTag
        if (limelightTable.getEntry("tv").getDouble(0) != 1.0) return;

        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (botpose.length < 7) return;

        // Validate that all values are real numbers (not NaN or Infinity)
        for (double v : botpose) {
            if (Double.isNaN(v) || Double.isInfinite(v)) return;
        }

        // Create a Pose2d from the vision data (x, y, yaw)
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));

        // Compensate for camera latency: the image was taken (latency) milliseconds ago
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);

        // Feed the vision measurement into the pose estimator
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /**
     * Returns true if the Limelight currently sees a valid AprilTag target.
     *
     * @return true if a target is visible
     */
    public boolean getLimelightHasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    /**
     * Returns the horizontal angle offset (in degrees) from the crosshair to the best target.
     * Negative = target is to the left, Positive = target is to the right.
     * Used by AlignToTag to auto-rotate toward the target.
     *
     * @return horizontal angle to target in degrees
     */
    public double getLimelightTx() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * Returns the AprilTag ID of the primary (best) target, or -1 if no target is visible.
     * Used by AutoShooter to verify we're aiming at our alliance's HUB tags.
     *
     * @return the AprilTag ID, or -1 if no target
     */
    public int getLimelightTid() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Calculates the 3D distance (in meters) from the camera to the primary target.
     * Uses the targetpose_cameraspace entry which provides the target's [x, y, z]
     * position relative to the camera. Returns -1.0 if no target is available.
     *
     * @return distance to target in meters, or -1.0 if no target
     */
    public double getLimelightTargetDistanceMeters() {
        double[] targetpose = limelightTable.getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[0]);
        if (targetpose.length < 3) return -1.0;
        return Math.sqrt(targetpose[0] * targetpose[0]
            + targetpose[1] * targetpose[1]
            + targetpose[2] * targetpose[2]);
    }

    /**
     * Publishes encoder values and sensor readings to SmartDashboard for debugging.
     * Called every 20ms from periodic().
     */
    protected void execute() {
        // Drive encoder velocities (m/s)
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        // Steering encoder positions (radians)
        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        // NavX heading (degrees)
        SmartDashboard.putNumber("Yaw", -navX.getYaw());

        // Current field-relativity status
        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);
    }
}
