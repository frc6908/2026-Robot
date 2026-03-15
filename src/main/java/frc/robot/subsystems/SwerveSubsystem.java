package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants; // Make sure this exists in Constants.java



import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveSubsystem extends SubsystemBase{
    public static boolean fieldRelativeStatus = true;
    public static boolean invertedControls = false;

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
        DrivetrainConstants.kPRotation,
        -1.0
    );
    private final SwerveModule backLeft = new SwerveModule(
        DrivetrainConstants.kBLDrive,
        DrivetrainConstants.kBLRotate,
        DrivetrainConstants.kBLCanCoder,
        DrivetrainConstants.kBLOffsetRad,
        DrivetrainConstants.bLIsInverted,
        DrivetrainConstants.kPRotation,
        -1.0
    );
    private final SwerveModule backRight = new SwerveModule(
        DrivetrainConstants.kBRDrive,
        DrivetrainConstants.kBRRotate,
        DrivetrainConstants.kBRCanCoder,
        DrivetrainConstants.kBROffsetRad,
        DrivetrainConstants.bRIsInverted,
        DrivetrainConstants.kPRotation
    );

    // navX
    private final AHRS navX;

    // Pose Estimator (Replaces standard Odometry)
    private final SwerveDrivePoseEstimator poseEstimator;

    // Vision Components (Limelight via NetworkTables)
    private final NetworkTable limelightTable;



    public SwerveSubsystem(){
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX.reset();
            }
            catch(Exception e){}
        }).start();

        // initialize rotation offsets
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        // reset encoders upon start
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        // --- Vision Initialization (Limelight) ---
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightName);

        HttpCamera cameraStream = new HttpCamera("LimelightStream",
            "http://10.69.8.11:5800/stream.mjpg");
        CameraServer.addCamera(cameraStream);

        Shuffleboard.getTab("Driver")
            .add("Limelight", cameraStream)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(4, 3);

        // --- Pose Estimator Initialization ---
        // Using DrivetrainConstants.SwerveDriveKinematics to match original odometry usage
        poseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.SwerveDriveKinematics,
            new Rotation2d(), // Initial Gyro
            getModulePositions(),
            new Pose2d() // Initial Pose
        );

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry 
            this::getRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    

    // method to stop modules
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Pose2d getPose(){
        // Return the estimated pose from the pose estimator (fused vision + encoders)
        return poseEstimator.getEstimatedPosition();
    }

    
    public ChassisSpeeds getRobotChassisSpeeds(){
        return DrivetrainConstants.SwerveDriveKinematics.toChassisSpeeds(getStates());
    }


    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-navX.getYaw());
    }


    public AHRS getNavX(){
        return navX;
    }

    public void resetHeading(){
        navX.reset();
    }

    public void resetOdometry(Pose2d pose){
        // Resets the pose estimator to a specific pose
        poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);

        SmartDashboard.putNumber("FL Set Position", desiredStates[0].angle.getRadians());
        SmartDashboard.putNumber("FR Set Position", desiredStates[1].angle.getRadians());
        SmartDashboard.putNumber("BL Set Position", desiredStates[2].angle.getRadians());
        SmartDashboard.putNumber("BR Set Position", desiredStates[3].angle.getRadians());
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad())),
        };
        return positions;
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    public void setFieldRelativity(boolean relativity){
        fieldRelativeStatus = relativity;
    }

    public void setInvertedControls(boolean inverted){
        invertedControls = inverted;
    }

    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative){
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        
        Logger.recordOutput("ChassisSpeeds", speeds);

        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);

        SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
    }

    @Override
    public void periodic(){
        super.periodic();
        
        // Update PoseEstimator with Encoders and Gyro
        poseEstimator.update(getHeading(), getModulePositions());

        // Update PoseEstimator with Vision
        updateVisionPose();

        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());
        
        execute();
    }

    /**
     * Updates the SwerveDrivePoseEstimator with vision measurements from Limelight.
     * Uses botpose_wpiblue which provides robot pose in the WPILib blue-origin coordinate frame.
     * The array format is [x, y, z, roll, pitch, yaw, total_latency_ms].
     */
    private void updateVisionPose() {
        if (limelightTable.getEntry("tv").getDouble(0) != 1.0) return;

        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        if (botpose.length < 7) return;
        for (double v : botpose) {
            if (Double.isNaN(v) || Double.isInfinite(v)) return;
        }

        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        double timestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000.0);
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    /** Returns true if the Limelight has a valid target. */
    public boolean getLimelightHasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    /** Returns the horizontal angle offset (degrees) to the best target. Negative = left, positive = right. */
    public double getLimelightTx() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /** Returns the AprilTag ID of the primary target, or -1 if no target. */
    public int getLimelightTid() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Returns the 3D distance (meters) from the camera to the primary target,
     * or -1.0 if no target is available.
     */
    public double getLimelightTargetDistanceMeters() {
        double[] targetpose = limelightTable.getEntry("targetpose_cameraspace")
            .getDoubleArray(new double[0]);
        if (targetpose.length < 3) return -1.0;
        return Math.sqrt(targetpose[0] * targetpose[0]
            + targetpose[1] * targetpose[1]
            + targetpose[2] * targetpose[2]);
    }

    protected void execute() {
        // encoder values
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        SmartDashboard.putNumber("Yaw", -navX.getYaw());

        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);
    }
}