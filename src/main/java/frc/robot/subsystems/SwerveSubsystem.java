package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;



import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;


import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * The SwerveSubsystem manages the entire swerve drivetrain -- all 4 modules working
 * together to move the robot.
 *
 * Key concepts:
 *   - FIELD-RELATIVE vs ROBOT-RELATIVE driving:
 *       Field-relative: pushing the joystick "up" always moves the robot toward the
 *       far end of the field, no matter which way the robot is facing. This is usually
 *       easier for drivers because it feels like controlling a video game character.
 *
 *       Robot-relative: pushing "up" moves the robot wherever its front is pointing.
 *       If the robot is turned sideways, "up" goes sideways on the field.
 *
 *   - ODOMETRY: the robot tracks its own position on the field using wheel encoders
 *     and the gyroscope. It can't see where it is, but by measuring how far each
 *     wheel has turned and which direction it's facing, it can estimate its (x, y)
 *     position. This is used by autonomous routines.
 *
 *   - NavX (AHRS): a gyroscope that measures which direction the robot is facing.
 *     "Yaw" is the rotation around the vertical axis (spinning left/right).
 */
public class SwerveSubsystem extends SubsystemBase{
    // Are we driving field-relative (true) or robot-relative (false)?
    // This is public static so other classes can read it easily.
    //
    // WANT TO CHANGE the default drive mode when the robot starts?
    //   Set this to false to start in robot-relative mode instead.
    public static boolean fieldRelativeStatus = true;

    // --- The 4 swerve modules (one for each corner) ---
    // Each module gets its CAN IDs, CANcoder offset, and PID values from Constants.
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

    // --- Gyroscope (NavX) ---
    // The AHRS (Attitude and Heading Reference System) is a gyroscope that tells us
    // which direction the robot is facing. We need this for field-relative driving
    // and for tracking the robot's position on the field.
    private final AHRS navX;

    // --- Odometry ---
    // Odometry combines wheel encoder data + gyro heading to estimate the robot's
    // position on the field. It's like a dead-reckoning GPS for the robot.
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DrivetrainConstants.SwerveDriveKinematics,
        new Rotation2d(),
        getModulePositions()
    );

    // A second kinematics object used by PathPlanner for autonomous driving.
    // This defines where each module is relative to the center of the robot (in meters).
    private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
        new Translation2d(.7112 / 2, 1.0 / 2),
        new Translation2d(.7112 / 2, -1.0 / 2),
        new Translation2d(-.7112 / 2, 1.0 / 2),
        new Translation2d(-.7112 / 2, -1.0 / 2)
    );


    /**
     * Constructor -- sets up the drivetrain when the robot boots.
     */
    public SwerveSubsystem(){
        // Create the NavX gyroscope (connected via the MXP SPI port on the RoboRIO)
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // The NavX takes about a second to fully boot up after being powered on.
        // We use a separate thread (a background task) to wait 1 second and then
        // reset it, so we don't freeze the rest of the robot code while waiting.
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX.reset();
                odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
            }
            catch(Exception e){}
        }).start();

        // Sync each module's motor encoder to its CANcoder so they all start
        // with the correct angle reading
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        backLeft.initRotationOffset();
        backRight.initRotationOffset();

        // Reset drive encoders to 0 (we haven't traveled any distance yet)
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        // --- PathPlanner Setup ---
        // PathPlanner is the tool we use to create autonomous paths.
        // AutoBuilder.configure() tells PathPlanner how to control our specific robot.
        RobotConfig config = null;
        try{
            // Load robot config (mass, dimensions, etc.) from the PathPlanner GUI settings
            config = RobotConfig.fromGUISettings();
         } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,              // How to get the robot's current position
            this::resetOdometry,        // How to reset position (used at the start of auto paths)
            this::getRobotChassisSpeeds,// How to get the robot's current velocity
            // How to actually drive the robot (PathPlanner tells us what speeds to use)
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            // PID controller for following the path -- separate PID for translation
            // (moving to the right spot) and rotation (facing the right direction)
            //
            // WANT TO TUNE how well the robot follows auto paths?
            //   - Robot drifts off the path? → increase Translation P value
            //   - Robot oscillates around the path? → decrease Translation P value
            //   - Robot doesn't face the right direction? → adjust Rotation P value
            //   - These are separate from the swerve module PID in Constants.java
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
            ),
            config,
            // This tells PathPlanner whether to mirror the path for the red alliance.
            // FRC fields are symmetric, so a path designed for blue side needs to be
            // flipped when we're on the red side.
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // This subsystem is "required" by PathPlanner commands
        );
    }


    /** Stops all 4 modules (all 8 motors). */
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Returns the robot's estimated position on the field as (x, y, angle). */
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    /**
     * Returns the robot's current velocity as a ChassisSpeeds object.
     * ChassisSpeeds combines forward speed, sideways speed, and rotation speed
     * into one object. Used by PathPlanner to know how fast we're currently going.
     */
    public ChassisSpeeds getRobotChassisSpeeds(){
        return kinematics.toChassisSpeeds(getStates());
    }

    /**
     * Drives the robot using robot-relative speeds (used by PathPlanner during auto).
     * Takes a ChassisSpeeds object and converts it into individual module states.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    /**
     * Returns which direction the robot is facing as a Rotation2d.
     * The negative sign flips the NavX reading to match the standard math
     * convention (counter-clockwise = positive).
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

    /** Returns the raw NavX object (in case other code needs direct access). */
    public AHRS getNavX(){
        return navX;
    }

    /** Resets the gyro heading to 0 degrees (whatever direction the robot is currently facing becomes "forward"). */
    public void resetHeading(){
        navX.reset();
    }

    /**
     * Resets the odometry to a specific position on the field.
     * Used at the start of autonomous to tell the robot "you are HERE."
     */
    public void resetOdometry(Pose2d pose){
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /**
     * Sends a desired state (speed + angle) to each of the 4 modules.
     * Also publishes the target angles to SmartDashboard for debugging.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);

        // Log the target angles to the dashboard so we can see what the code
        // is trying to do vs what the modules are actually doing
        SmartDashboard.putNumber("FL Set Position", desiredStates[0].angle.getRadians());
        SmartDashboard.putNumber("FR Set Position", desiredStates[1].angle.getRadians());
        SmartDashboard.putNumber("BL Set Position", desiredStates[2].angle.getRadians());
        SmartDashboard.putNumber("BR Set Position", desiredStates[3].angle.getRadians());
    }

    /**
     * Returns the current position of each module (distance traveled + current angle).
     * Used by odometry to calculate the robot's position on the field.
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

    /** Returns the current state (speed + angle) of each module. */
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    /** Switches between field-relative and robot-relative driving. */
    public void setFieldRelativity(boolean relativity){
        fieldRelativeStatus = relativity;
    }

    /**
     * The main drive method -- this is what actually makes the robot move!
     *
     * Takes desired forward, strafe, and rotation speeds and sends them to
     * all 4 swerve modules.
     *
     * @param forward         forward/backward speed in m/s (positive = forward)
     * @param strafe          left/right speed in m/s (positive = left)
     * @param rotation        rotation speed in rad/s (positive = counter-clockwise)
     * @param isFieldRelative if true, directions are relative to the field;
     *                        if false, relative to the robot
     */
    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative){
        // Convert our desired movement into ChassisSpeeds.
        // If field-relative, we need the gyro heading to translate field directions
        // into robot directions. If robot-relative, we just use the values directly.
        ChassisSpeeds speeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
            : new ChassisSpeeds(forward, strafe, rotation);

        // "Discretize" compensates for the fact that we only update every 20ms.
        // Without this, the robot can drift slightly when driving and rotating
        // at the same time because the rotation happens between updates.
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Log chassis speeds to AdvantageKit for replay/analysis
        Logger.recordOutput("ChassisSpeeds", speeds);

        // Convert the overall robot speed into individual wheel speeds and angles.
        // This is the magic of swerve -- kinematics figures out what each wheel
        // needs to do to achieve the desired overall robot movement.
        SwerveModuleState[] states = DrivetrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);

        // If the math asks a wheel to go faster than physically possible,
        // desaturate scales all wheels down proportionally so the motion
        // stays correct (just slower).
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);
        setModuleStates(states);

        // Log individual module speeds to SmartDashboard for debugging
        SmartDashboard.putNumber("FL Set Speed", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("FR Set Speed", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("BL Set Speed", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("BR Set Speed", states[3].speedMetersPerSecond);
    }

    /**
     * Called every 20ms by the CommandScheduler. Updates odometry (position tracking)
     * and logs data for debugging and AdvantageKit replay.
     */
    @Override
    public void periodic(){
        super.periodic();

        // Update odometry with the latest heading and module positions.
        // This is how the robot keeps track of where it is on the field.
        odometry.update(getHeading(), getModulePositions());

        // Log data to AdvantageKit (for post-match analysis and replay)
        Logger.recordOutput("MyStates", getStates());
        Logger.recordOutput("NavX Heading", getHeading());
        Logger.recordOutput("Odometry Pose", getPose());

        // Update SmartDashboard with debug info
        execute();
    }

    /**
     * Publishes encoder values and heading to SmartDashboard so we can monitor
     * the drivetrain from the laptop during testing.
     *
     * WANT TO ADD more data to the dashboard? Add SmartDashboard.putNumber() or
     * SmartDashboard.putBoolean() calls here. They'll update every 20ms.
     */
    protected void execute() {
        // Drive motor speeds (m/s) -- useful for checking if all wheels are working
        SmartDashboard.putNumber("FL Drive Encoder", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FR Drive Encoder", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("BL Drive Encoder", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("BR Drive Encoder", backRight.getDriveVelocity());

        // Steering angles (radians) -- useful for checking if wheels are pointing correctly
        SmartDashboard.putNumber("FL Angle Position", frontLeft.getRotationPosition());
        SmartDashboard.putNumber("FR Angle Position", frontRight.getRotationPosition());
        SmartDashboard.putNumber("BL Angle Position", backLeft.getRotationPosition());
        SmartDashboard.putNumber("BR Angle Position", backRight.getRotationPosition());

        // Current robot heading from the gyro
        SmartDashboard.putNumber("Yaw", -navX.getYaw());

        // Show whether we're in field-relative or robot-relative mode
        SmartDashboard.putBoolean("Field Realtive", fieldRelativeStatus);
    }
}
