package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

/**
 * Represents ONE swerve module (one corner of the robot).
 *
 * Each swerve module has:
 *   - A DRIVE motor: spins the wheel to move the robot forward/backward
 *   - A ROTATION motor: turns the wheel left/right to change direction
 *   - A CANcoder: an absolute encoder that always knows the exact angle of the
 *     wheel, even after a power cycle. This is important because the relative
 *     encoders built into the motors lose their position when power is lost.
 *
 * The robot has 4 of these modules (front-left, front-right, back-left, back-right).
 * Together they form the swerve drivetrain.
 *
 * You usually don't need to edit this file unless you're:
 *   - Changing how the motors behave (brake vs coast mode) → configureMotor()
 *   - Re-enabling the wheel optimization feature → uncomment the optimize() call in setState()
 *   - Tuning PID values → change them in Constants.DrivetrainConstants instead
 */
public class SwerveModule extends SubsystemBase {
    // The two motors on this module
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    // Built-in encoders inside each motor (these are "relative" -- they count
    // from wherever they were when the robot turned on)
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    // The CANcoder is an "absolute" encoder mounted on the steering shaft.
    // Unlike the motor's built-in encoder, this one always knows the true angle
    // of the wheel, even after the robot restarts.
    private final CANcoder canCoder;
    // Each module's CANcoder has a slightly different zero point due to how
    // it was mounted. This offset corrects for that.
    private final double canCoderOffsetRadians;

    // PID controllers -- these are the math that smoothly controls motor output.
    // Instead of just slamming the motor to full speed, PID gradually adjusts
    // the power to reach the target without overshooting.
    private final PIDController rotationPIDController;
    private final PIDController drivePIDController;

    /**
     * Creates a new swerve module.
     *
     * @param driveMotorID        CAN ID of the drive motor controller
     * @param rotationMotorID     CAN ID of the rotation motor controller
     * @param canCoderID          CAN ID of the CANcoder
     * @param canCoderOffsetRadians  calibration offset for the CANcoder (in radians)
     * @param isDriveInverted     whether the drive motor direction is flipped
     * @param x                   the P value for the rotation PID controller
     */
    public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID,
      double canCoderOffsetRadians,
      boolean isDriveInverted,
      double x
    ) {
      // Create the motor controller objects.
      // kBrushless means we're using NEO motors (brushless motors are more
      // efficient and last longer than brushed motors).
      driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

      // Configure both motors with the right settings (inversion, brake mode,
      // and conversion factors so encoders report in meters and radians)
      //
      // WANT THE ROBOT TO COAST (slide) when you let go of the sticks?
      //   Change IdleMode.kBrake to IdleMode.kCoast below.
      //   Brake = robot stops quickly. Coast = robot glides to a stop.
      configureMotor(driveMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.drivePositionConversionFactor,
                      DrivetrainConstants.driveVelocityConversionFactor
      );
      configureMotor(rotationMotor,
                      isDriveInverted,
                      IdleMode.kBrake,
                      DrivetrainConstants.rotationPositionConversionFactor,
                      DrivetrainConstants.rotationVelocityConversionFactor
      );
      // Clear any old error codes from the motor controllers
      driveMotor.clearFaults();
      rotationMotor.clearFaults();

      // Set up PID controllers for both motors
      drivePIDController = new PIDController(DrivetrainConstants.kPDrive, DrivetrainConstants.kIDrive, DrivetrainConstants.kDDrive);

      // Rotation PID has some extra setup:
      rotationPIDController = new PIDController(x, DrivetrainConstants.kIRotation, DrivetrainConstants.kDRotation);
      // Tolerance = how close is "close enough" to the target angle
      rotationPIDController.setTolerance(DrivetrainConstants.kToleranceRotation);
      // enableContinuousInput tells the PID that -PI and +PI are the same angle
      // (like how 359 degrees and -1 degree are basically the same).
      // Without this, the wheel might spin 350 degrees the long way around
      // instead of just 10 degrees the short way.
      rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // Set up the CANcoder and grab references to the built-in motor encoders
      canCoder = new CANcoder(canCoderID);
      this.canCoderOffsetRadians = canCoderOffsetRadians;
      configureCanCoder();
      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    /**
     * Applies configuration to a SparkMax motor controller.
     *
     * @param motorController        the SparkMax to configure
     * @param isInverted             flip the motor direction (true = reversed)
     * @param idleMode               what happens when no power is applied:
     *                               kBrake = motor resists movement (like putting a car in park)
     *                               kCoast = motor spins freely (like putting a car in neutral)
     * @param positionConversionFactor  multiplier to convert motor rotations to useful units (meters or radians)
     * @param velocityConversionFactor  multiplier to convert RPM to useful units (m/s or rad/s)
     */
    public void configureMotor(
      SparkMax motorController,
      boolean isInverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .inverted(isInverted)
          .idleMode(idleMode);
        // Set conversion factors so encoder readings come back in meters/radians
        // instead of raw motor rotations
        config.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(velocityConversionFactor);
        // Apply the config. kResetSafeParameters clears any old settings first,
        // and kPersistParameters saves this config so it survives power cycles.
        motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Configures the CANcoder (the absolute angle sensor on the steering shaft).
     * Sets it to:
     *   - Report values from 0 to 1 (fraction of a full rotation)
     *   - Count counter-clockwise as the positive direction
     */
    public void configureCanCoder(){
      CANcoderConfiguration config = new CANcoderConfiguration();

      // Setting the discontinuity point to 1 means the sensor reports 0.0 to 1.0
      // (one full rotation) before wrapping back to 0.
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      // Counter-clockwise is positive (standard math convention)
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      canCoder.getConfigurator().apply(config);
    }

    /**
     * Tells this module what speed and angle to go to.
     *
     * This is the main method that gets called every 20ms to control the module.
     * It uses PID to smoothly steer the wheel to the target angle, and sets the
     * drive motor to the target speed.
     *
     * @param state  the desired speed (m/s) and angle for this module
     */
    public void setState(SwerveModuleState state){
      // If the desired speed is basically zero, just stop the motors.
      // This prevents the wheels from jittering when the robot should be still.
      if (Math.abs(state.speedMetersPerSecond) < 0.0001){
        stop();
        return;
      }

      // WANT TO RE-ENABLE module optimization (prevents wheels from turning more than 90 degrees)?
      //   Uncomment this line:
      // state = optimize(state, getState().angle);

      // Use PID to calculate how much power the rotation motor needs to reach
      // the target angle. getCANCoderRad() is where we are now, state.angle is
      // where we want to be.
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), state.angle.getRadians()));

      // Set the drive motor speed using PID
      driveMotor.set(drivePIDController.calculate(state.speedMetersPerSecond));
    }

    /**
     * Optimization: prevents the wheel from rotating more than 90 degrees.
     *
     * For example, if the wheel is pointing at 0 degrees and we want it at 170,
     * instead of rotating 170 degrees, we can rotate -10 degrees and reverse
     * the drive motor. Same result, much less wheel movement.
     *
     * (Currently commented out in setState -- can be re-enabled if needed)
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90){
        // Flip the drive direction and rotate to the opposite angle
        return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
      }
      else{
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    /**
     * Returns the current state of this module (how fast the wheel is spinning
     * and what angle it's pointing at).
     */
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
    }

    /**
     * Syncs the rotation motor's built-in encoder to the CANcoder's value.
     * Called once at startup so the relative encoder starts at the correct position.
     */
    public void initRotationOffset() {
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /**
     * Reads the CANcoder and returns the wheel's current angle in radians,
     * with the calibration offset applied.
     *
     * The CANcoder reports a value from 0 to 1 (fraction of a full turn).
     * We multiply by 2*PI to convert to radians, then subtract the offset
     * so that 0 radians = wheel pointing straight ahead.
     */
    public double getCANCoderRad() {
      double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
      double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
      return angle % (2 * Math.PI);
    }

    /**
     * Directly sets the rotation motor to aim for a specific angle using PID.
     */
    public void setRotationMotorAnglePID(double angleRad) {
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), angleRad));
    }

    /**
     * Resets encoders to known positions:
     *   - Drive encoder goes to 0 (we've traveled 0 meters from this point)
     *   - Rotation encoder syncs to the CANcoder (which always knows the true angle)
     */
    public void resetEncoder() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /** Returns the drive wheel speed in meters/second. */
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    /** Returns the steering speed in radians/second. */
    public double getRotationVelocity(){
      return rotationEncoder.getVelocity();
    }

    /** Stops both motors on this module immediately. */
    public void stop(){
      driveMotor.stopMotor();
      rotationMotor.stopMotor();
    }

    /** Returns how far the drive wheel has traveled in meters (since last reset). */
    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }

    /** Returns the current wheel angle in radians (reads from the CANcoder). */
    public double getRotationPosition() {
      return getCANCoderRad();
    }

    @Override
    public void periodic() {
      // This method is called every 20ms by the CommandScheduler.
      // Currently empty -- could be used to log module-specific data.
    }
}
