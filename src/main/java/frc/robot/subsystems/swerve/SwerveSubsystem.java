package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DrivetrainConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * The swerve drivetrain -- this controls how the robot moves.
 *
 * A swerve drive has 4 wheels that can each point in any direction AND spin at any
 * speed. This lets the robot drive in any direction while also spinning.
 *
 * This subsystem uses the IO pattern: instead of talking to hardware directly,
 * it talks to IO interfaces (GyroIO, VisionIO, SwerveModuleIO). On the real
 * robot, these are backed by real hardware. In simulation, they use maple-sim
 * physics. The subsystem code is identical either way.
 *
 * This class manages:
 *   - Four SwerveModules (one per corner: FL, FR, BL, BR)
 *   - A gyroscope (via GyroIO) that knows which way the robot is facing
 *   - A Pose Estimator that figures out WHERE the robot is on the field
 *   - Vision (via VisionIO) for AprilTag-based position correction
 *   - PathPlanner for auto routines
 *
 * WANT TO CHANGE how the robot drives? Look at the drive() method.
 * WANT TO CHANGE camera stuff? Look at updateVisionPose().
 * WANT TO CHANGE auto path following? Look at the AutoBuilder.configure() call.
 */
public class SwerveSubsystem extends SubsystemBase {

    /**
     * Are we driving field-relative (true) or robot-relative (false)?
     * Field-relative: "up" on the stick always goes toward the far wall.
     * Robot-relative: "up" on the stick goes wherever the robot's front is pointing.
     */
    public static boolean fieldRelativeStatus = true;

    // ========================
    // IO LAYERS
    // ========================

    /** Gyroscope IO layer. */
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /** Vision IO layer. */
    private final VisionIO visionIO;
    private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

    // ========================
    // SWERVE MODULES
    // ========================

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    /**
     * Tracks the robot's position on the field by combining wheel movement data,
     * gyro heading, and vision camera readings.
     */
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * Constructor -- sets up all IO layers and configures PathPlanner.
     *
     * @param gyroIO       the gyro IO implementation (real or sim)
     * @param visionIO     the vision IO implementation (real or sim)
     * @param flIO         front-left module IO
     * @param frIO         front-right module IO
     * @param blIO         back-left module IO
     * @param brIO         back-right module IO
     */
    public SwerveSubsystem(
        GyroIO gyroIO,
        VisionIO visionIO,
        SwerveModuleIO flIO,
        SwerveModuleIO frIO,
        SwerveModuleIO blIO,
        SwerveModuleIO brIO
    ) {
        this.gyroIO = gyroIO;
        this.visionIO = visionIO;

        // Create modules with their IO layers and names for logging
        frontLeft = new SwerveModule(flIO, "FL", DrivetrainConstants.kPRotation);
        frontRight = new SwerveModule(frIO, "FR", DrivetrainConstants.kPRotation);
        backLeft = new SwerveModule(blIO, "BL", DrivetrainConstants.kPRotation);
        backRight = new SwerveModule(brIO, "BR", DrivetrainConstants.kPRotation);

        // --- Pose Estimator Initialization ---
        // Start with a known position (0,0) facing forward (0 degrees).
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SwerveDriveKinematics,
            new Rotation2d(),
            getModulePositions(),
            new Pose2d()
        );

        // --- PathPlanner Configuration ---
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),
                    new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
        );
    }

    /** Stops all four swerve modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Returns where the robot thinks it is on the field. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns how fast the robot is currently moving and spinning. */
    public ChassisSpeeds getRobotChassisSpeeds() {
        return DrivetrainConstants.SwerveDriveKinematics.toChassisSpeeds(getStates());
    }

    /** Drives the robot at the given speeds. Used by PathPlanner during auto mode. */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivetrainConstants.maxVelocity);
        setModuleStates(moduleStates);
    }

    /** Returns which direction the robot is facing (from the gyro). */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyroInputs.yawDegrees);
    }

    /** Resets the gyro so the robot's current direction becomes "forward" (0 degrees). */
    public void resetHeading() {
        gyroIO.reset();
    }

    /** Tells the robot "you are HERE on the field." Used at the start of auto. */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /** Tells each wheel how fast to spin and which direction to point. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
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

    /** Returns the current position of each swerve module. */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

    /** Returns the current state of each swerve module. */
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    /** Sets whether driving should be field-relative or robot-relative. */
    public void setFieldRelativity(boolean relativity) {
        fieldRelativeStatus = relativity;
    }

    /**
     * The main drive method -- this is what actually makes the robot move.
     *
     * @param forward  forward/backward speed in m/s (positive = forward)
     * @param strafe   left/right speed in m/s (positive = left)
     * @param rotation spin speed in rad/s (positive = counter-clockwise)
     * @param isFieldRelative true = field-relative, false = robot-relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);

        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        Logger.recordOutput("ChassisSpeeds", speeds);

        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);
        setModuleStates(states);

        SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
    }

    /** Runs every 20ms. Updates all IO layers, pose estimator, and telemetry. */
    @Override
    public void periodic() {
        super.periodic();

        // Update all IO layers
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        visionIO.updateInputs(visionInputs);
        Logger.processInputs("Vision", visionInputs);

        frontLeft.updateInputs();
        frontRight.updateInputs();
        backLeft.updateInputs();
        backRight.updateInputs();

        // Update position using wheel + gyro data
        poseEstimator.update(getHeading(), getModulePositions());

        // Update position using vision data
        updateVisionPose();

        // Log data for AdvantageKit
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());

        // Publish debug telemetry
        execute();
    }

    /**
     * Uses vision data to update the robot's position on the field.
     * Does nothing if no target is visible or the data looks bad.
     */
    private void updateVisionPose() {
        if (!visionInputs.hasTarget) return;

        double[] botpose = visionInputs.botpose;
        if (botpose.length < 7) return;

        for (double v : botpose) {
            if (Double.isNaN(v) || Double.isInfinite(v)) return;
        }

        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);

        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /** Returns true if the vision camera currently sees a valid AprilTag target. */
    public boolean getLimelightHasTarget() {
        return visionInputs.hasTarget;
    }

    /** Returns horizontal offset to the target in degrees. */
    public double getLimelightTx() {
        return visionInputs.tx;
    }

    /** Returns which AprilTag number the camera is looking at, or -1 if none. */
    public int getLimelightTid() {
        return visionInputs.targetId;
    }

    /** Returns how far away the target is in meters, or -1.0 if no target. */
    public double getLimelightTargetDistanceMeters() {
        return visionInputs.targetDistanceMeters;
    }

    /** Publishes encoder values and sensor readings to SmartDashboard for debugging. */
    protected void execute() {
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        SmartDashboard.putNumber("Yaw", gyroInputs.yawDegrees);

        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);
    }
}
