// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * All robot-wide numbers live here. Think of this as the robot's "settings" file.
 *
 * We keep everything in one place so if you need to change a motor ID, PID value,
 * or speed limit, you only change it here -- not scattered across 10 different files.
 *
 * Each inner class groups related constants together (controller stuff, intake stuff,
 * drivetrain stuff, etc.).
 *
 * THIS IS THE FIRST FILE TO LOOK AT when you want to change robot behavior.
 * Most tweaks (speed, sensitivity, motor IDs, PID tuning) happen here.
 *
 * WANT TO CHANGE robot speed? Look in DrivetrainConstants.
 * WANT TO CHANGE controller sensitivity? Look in OperatorConstants.
 * WANT TO CHANGE a motor port? Look in the appropriate mechanism class (IntakeConstants, etc.).
 * WANT TO CHANGE shooter behavior? Look in ShooterConstants.
 */
public final class Constants {

  /**
   * Settings related to the driver and operator Xbox controllers.
   *
   * "Ports" here are the USB port numbers assigned in the FRC Driver Station.
   * Port 0 is typically the driver controller, port 1 is the operator controller.
   *
   * "Deadbands" prevent tiny accidental joystick movements from moving the robot.
   * If the joystick value is smaller than the deadband, the code treats it as zero.
   * Think of it like a "dead zone" around the center of the joystick.
   *
   * WANT TO CHANGE controller sensitivity? Increase the deadband values to require
   * bigger joystick movements before the robot responds. Decrease them for more
   * sensitive control (but the robot might drift if set too low).
   */
  public static class OperatorConstants {

    /* ================ */
    /* CONTROLLER PORTS */
    /* ================ */

    /** USB port for the driver's Xbox controller (the person who drives the robot). */
    public static final int kDriverControllerPort = 0;

    /** USB port for the operator's Xbox controller (the person who controls mechanisms). */
    public static final int kOperatorControllerPort = 1;

    /* ========= */
    /* Deadbands */
    /* ========= */

    /**
     * Forward/backward deadband. Joystick values smaller than 0.1 (10%) are
     * treated as zero. Prevents the robot from creeping when the stick isn't
     * perfectly centered.
     */
    public static final double xDeadband = 0.1;

    /**
     * Left/right (strafe) deadband. Same idea as xDeadband but for sideways movement.
     */
    public static final double yDeadband = 0.1;

    /**
     * Rotation deadband. Set higher (0.3 = 30%) because small accidental rotations
     * are more disorienting than small translational movements. The robot won't spin
     * unless the driver deliberately pushes the right stick.
     */
    public static final double rDeadband = 0.3;
  }


  /**
   * Settings for the intake mechanism -- the rollers that suck game pieces into the robot.
   *
   * The intake uses a single SparkMax motor controller running a brushless (NEO) motor.
   * Positive speed = intake (suck in), negative speed = outtake (spit out).
   *
   * WANT TO CHANGE intake/outtake speed? Change intakeSpeed / outtakeSpeed below.
   * WANT TO CHANGE the motor's CAN ID? Change ioSparkPort (must match the actual hardware).
   */
  public static class IntakeConstants {

    /** CAN ID of the intake motor controller (SparkMax). Must match the ID flashed to the controller. */
    public static final int ioSparkPort = 40;

    /** Speed to run the intake rollers (0.0 to 1.0). 0.75 = 75% power. */
    public static final double intakeSpeed = 0.75;

    /**
     * Speed to run the outtake (negative = reverse direction).
     * -0.75 = 75% power in reverse, which spits the game piece out.
     */
    public static final double outtakeSpeed = -0.75;

    /**
     * Maximum current the motor is allowed to draw (in Amps).
     * This protects the motor from burning out if it stalls (e.g., game piece jammed).
     * 35A is a safe limit for NEO motors in most FRC applications.
     */
    public static final int currentLimit = 35;
  }

  /**
   * Settings for the climbing mechanism -- the motor that lifts the robot onto the chain
   * at the end of a match.
   *
   * Uses a single SparkMax motor controller.
   *
   * WANT TO CHANGE climb speed? The speed is set directly in the Climb/ClimbDown commands.
   * WANT TO CHANGE the motor's CAN ID? Change climbSparkPort1 below.
   */
  public static class ClimbConstants {

    /** CAN ID of the climb motor controller (SparkMax). */
    public static final int climbSparkPort1 = 45;

    /**
     * Maximum current the climb motor can draw (in Amps).
     * Climbing puts heavy load on the motor, so the current limit protects it.
     */
    public static final int currentLimit = 35;
  }


  /**
   * Settings for the shooter mechanism -- the dual-motor flywheel + kicker that launches
   * game pieces (FUEL) into the HUB.
   *
   * The shooter has THREE motors:
   *   - Two shooter motors (CAN IDs 42 & 43) that spin up a flywheel. Both run at the
   *     same speed to launch the game piece.
   *   - One kicker motor (CAN ID 44) that pushes the game piece into the spinning flywheel.
   *     Think of the kicker as a "feeder" -- the flywheel spins up first, then the kicker
   *     shoves the ball in.
   *
   * The distance-to-speed lookup table (kDistanceToSpeedMap) is the key to auto-shooting:
   * the robot uses the Limelight camera to measure distance to the HUB, then interpolates
   * the correct flywheel speed from this table. For example, if the robot is 3.0m away,
   * the code will automatically pick a speed between the 2.5m and 4.0m entries.
   *
   * WANT TO CHANGE shooter speed? Change shooterSpeed for manual shooting, or update the
   * lookup table entries for auto-shooting.
   * WANT TO CHANGE motor CAN IDs? Update shooterSparkPort1, shooterSparkPort2, kickerSparkPort.
   */
  public static class ShooterConstants {

    /** CAN ID of the first shooter flywheel motor (SparkMax). */
    public static final int shooterSparkPort1 = 42;

    /** CAN ID of the second shooter flywheel motor (SparkMax). */
    public static final int shooterSparkPort2 = 43;

    /** CAN ID of the kicker/feeder motor (SparkMax). Pushes game pieces into the flywheel. */
    public static final int kickerSparkPort = 44;

    /** Maximum current any shooter motor can draw (in Amps). */
    public static final int currentLimit = 35;

    /**
     * Default shooter speed for manual (constant-speed) shooting.
     * 0.75 = 75% power. Used by the Shooter command when the driver presses B.
     */
    public static final double shooterSpeed = 0.75;

    // --- FRC 2026 REBUILT CONSTANTS ---

    /**
     * AprilTag IDs located on the RED alliance's HUB (the central scoring structure).
     * The AutoShooter command checks if the Limelight sees one of these tags to know
     * it's looking at the correct target.
     */
    public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

    /**
     * AprilTag IDs located on the BLUE alliance's HUB.
     * Same purpose as kRedHubTags, but for the other alliance.
     */
    public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

    /**
     * LOOKUP TABLE: Distance (Meters) -> Shooter Speed % (0.0 to 1.0)
     *
     * This is an "interpolating" map. That means if the robot is at a distance
     * between two entries, it will automatically calculate the right speed by
     * drawing a straight line between the two nearest points.
     *
     * Example: At 1.75m (between 1.0m and 2.5m entries), the code interpolates
     * a speed between 0.35 and 0.55.
     *
     * WANT TO TUNE? Measure shooting accuracy at known distances on the field,
     * then update these values. Add more entries for finer control.
     *
     * MEASURE THESE ON THE FIELD! The values below are starting points.
     */
    public static final InterpolatingDoubleTreeMap kDistanceToSpeedMap = new InterpolatingDoubleTreeMap();
    static {
        // Format: .put(Distance_Meters, Speed_Percent);

        kDistanceToSpeedMap.put(1.0, 0.35); // Close range (Fender shot -- right up against the HUB)
        kDistanceToSpeedMap.put(2.5, 0.55); // Mid range
        kDistanceToSpeedMap.put(4.0, 0.75); // Long range
        kDistanceToSpeedMap.put(6.0, 0.95); // Cross-field (maximum distance shot)
    }
  }

  /**
   * Settings for the Limelight vision camera.
   *
   * The Limelight is a smart camera that detects AprilTags on the field. It communicates
   * with the robot via NetworkTables (a shared data table over the network). The robot
   * reads values like "tx" (horizontal angle to target) and "tid" (tag ID) to aim and
   * estimate distance.
   *
   * WANT TO CHANGE which Limelight to use? If you renamed your Limelight in the web
   * interface, update kLimelightName to match.
   */
  public static class VisionConstants {

    /**
     * NetworkTables name for the Limelight camera. This must match the name configured
     * in the Limelight's web interface (default is "limelight"). If you have multiple
     * Limelights, each one needs a unique name.
     */
    public static final String kLimelightName = "limelight";
  }

  /**
   * Settings for the swerve drivetrain -- the 4-wheel drive system where each wheel
   * can independently steer and drive.
   *
   * This is the most complex section because swerve drive has a LOT of configuration:
   *   - Physical measurements (wheelbase, track width, wheel size)
   *   - Conversion factors (translating motor rotations into real-world distances)
   *   - Speed limits (how fast the robot can go)
   *   - CAN IDs for all 12 motor controllers and 4 encoders (3 per module x 4 modules)
   *   - PID tuning values for both driving and steering
   *
   * WANT TO CHANGE max speed? Look at maxVelocity and kTeleDriveMaxSpeed.
   * WANT TO CHANGE PID tuning? Look at kPDrive, kPRotation, etc. at the bottom.
   * WANT TO SWAP a motor? Find the correct module (FL/FR/BL/BR) and update its CAN IDs.
   */
  public static class DrivetrainConstants {

    /* ==================== */
    /* PHYSICAL DIMENSIONS  */
    /* ==================== */

    /**
     * Distance between the centers of the front and back wheels (in meters).
     * 21 inches = 0.5334 meters. Measured center-to-center.
     * WANT TO CHANGE? Only if you physically move the wheels on the chassis.
     */
    public static final double wheelBase = Units.inchesToMeters(21);

    /**
     * Distance between the centers of the left and right wheels (in meters).
     * 21 inches = 0.5334 meters. Same as wheelBase for our square chassis.
     */
    public static final double trackWidth = Units.inchesToMeters(21);

    /**
     * Diameter of the drive wheels (in meters). 4-inch wheels are standard for swerve.
     * Used in conversion factor calculations.
     */
    public static final double wheelDiameter = Units.inchesToMeters(4.0);

    /**
     * Swerve drive kinematics -- tells WPILib where each wheel is relative to the
     * robot's center. WPILib uses this to calculate how fast each wheel needs to spin
     * and what angle it needs to point at for any given robot movement.
     *
     * The Translation2d values are (x, y) positions of each module:
     *   - +X = front of the robot
     *   - +Y = left side of the robot
     *
     * The order matters! It must match the order you pass module states everywhere else.
     */
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // back right (+,-)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back left (-,-)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    /**
     * Converts motor rotations into meters traveled by the wheel.
     *
     * The math: (1 rev / Gear Ratio) * (wheel circumference)
     *         = (2 * PI * wheel_radius) / Gear_Ratio
     *
     * This factor is pre-calculated. If you change the gear ratio or wheel size,
     * you'll need to recalculate this value.
     *
     * WANT TO CHANGE? Recalculate: (Math.PI * wheelDiameter) / gearRatio
     */
    public static final double drivePositionConversionFactor = 0.0472867872;

    /**
     * Converts motor RPM into meters per second (m/s).
     * Simply the position factor divided by 60 (since RPM = revolutions per MINUTE
     * and we want per SECOND).
     */
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    /**
     * Converts motor rotations into radians traveled by the steering mechanism.
     *
     * The math: (1 rev / Gear Ratio) * (2 * PI radians per revolution)
     *         = (2 * PI) / Gear_Ratio
     */
    public static final double rotationPositionConversionFactor = 0.29321531433;

    /**
     * Converts motor RPM into radians per second for the steering mechanism.
     */
    public static final double rotationVelocityConversionFactor = rotationPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    /**
     * Maximum translational velocity the robot can achieve (in meters per second).
     * This is the physical limit of the drivetrain. 4 m/s is roughly 9 mph.
     *
     * WANT TO CHANGE? Increasing beyond the motor's capability won't make it faster.
     * Decreasing it will cap the robot's top speed (useful for practice/safety).
     */
    public static final double maxVelocity = 4; // m/s

    /**
     * Maximum translational acceleration (in meters per second squared).
     * Controls how quickly the robot can speed up. Higher = snappier but jerkier.
     * Lower = smoother but slower to respond.
     */
    public static final double maxAcceleration = 7; // m/s^2

    /**
     * Maximum rotational velocity (in radians per second).
     * 2*PI rad/s = one full rotation per second. This is how fast the robot can spin.
     */
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/s

    /**
     * Maximum rotational acceleration (in radians per second squared).
     * Controls how quickly the robot can start or stop spinning.
     */
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/s^2

    /**
     * Maximum teleop driving speed (in meters per second).
     * This is intentionally lower than maxVelocity to give drivers more control.
     * 7.5/4.0 = 1.875 m/s (about 4.2 mph).
     *
     * WANT TO CHANGE? Increase for a faster robot in teleop, decrease for more control.
     */
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // meters/sec

    /**
     * Maximum teleop rotation speed (in radians per second).
     * 3 rad/s is about half a rotation per second -- fast enough to turn quickly
     * but not so fast that the driver loses control.
     */
    public static final double kTeleDriveMaxAngularSpeed = 3; // rad/sec


    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * Each swerve module has three devices on the CAN bus:
     *   1. Drive motor (SparkMax) -- spins the wheel forward/backward
     *   2. Rotation motor (SparkMax) -- steers the wheel left/right
     *   3. CANcoder -- absolute encoder that always knows the wheel's angle,
     *      even after a power cycle
     *
     * The "offset" is a calibration value. When the wheel is physically pointing
     * straight forward, the CANcoder might read some non-zero value. The offset
     * compensates for this so the code knows "straight" = 0 radians.
     *
     * WANT TO CALIBRATE? Point all wheels straight forward, read each CANcoder's
     * raw value, and put that value as the offset.
     */

    // Front Left Module
    /** CAN ID of the front-left drive motor (SparkMax). */
    public static final int kFLDrive = 4;
    /** CAN ID of the front-left rotation/steering motor (SparkMax). */
    public static final int kFLRotate = 7;
    /** CAN ID of the front-left CANcoder (absolute encoder). */
    public static final int kFLCanCoder = 14;
    /** Offset for the front-left CANcoder (in radians). Calibrate with wheels pointing forward. */
    public static final double kFLOffsetRad = 0.042969 * 2 * Math.PI;
    /** Whether the front-left drive motor is inverted. */
    public static final boolean fLIsInverted = true;

    // Front Right Module
    /** CAN ID of the front-right drive motor (SparkMax). */
    public static final int kFRDrive = 5;
    /** CAN ID of the front-right rotation/steering motor (SparkMax). */
    public static final int kFRRotate = 3;
    /** CAN ID of the front-right CANcoder (absolute encoder). */
    public static final int kFRCanCoder = 13;
    /** Offset for the front-right CANcoder (in radians). */
    public static final double kFROffsetRad = 0.028564 * 2 * Math.PI;
    /** Whether the front-right drive motor is inverted. */
    public static final boolean fRIsInverted = true;

    // Back Left Module
    /** CAN ID of the back-left drive motor (SparkMax). */
    public static final int kBLDrive = 6;
    /** CAN ID of the back-left rotation/steering motor (SparkMax). */
    public static final int kBLRotate = 1;
    /** CAN ID of the back-left CANcoder (absolute encoder). */
    public static final int kBLCanCoder = 12;
    /** Offset for the back-left CANcoder (in radians). */
    public static final double kBLOffsetRad = 0.027466 * 2 * Math.PI;
    /** Whether the back-left drive motor is inverted. */
    public static final boolean bLIsInverted = true;

    // Back Right Module
    /** CAN ID of the back-right drive motor (SparkMax). */
    public static final int kBRDrive = 8;
    /** CAN ID of the back-right rotation/steering motor (SparkMax). */
    public static final int kBRRotate = 2;
    /** CAN ID of the back-right CANcoder (absolute encoder). */
    public static final int kBRCanCoder = 11;
    /** Offset for the back-right CANcoder (in radians). */
    public static final double kBROffsetRad = 0.340698 * 2 * Math.PI;
    /** Whether the back-right drive motor is inverted. */
    public static final boolean bRIsInverted = true;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */

    /*
     * PID stands for Proportional-Integral-Derivative. It's a control algorithm
     * that continuously adjusts motor output to reach a target value.
     *
     * - P (Proportional): "How hard do I push based on how far away I am?"
     *   Higher P = more aggressive correction, but can overshoot and oscillate.
     * - I (Integral): "Am I stuck? Let me push harder over time."
     *   Usually 0 for FRC drivetrain use -- we don't need it.
     * - D (Derivative): "Am I approaching too fast? Let me ease off."
     *   Helps dampen oscillation. A little goes a long way.
     *
     * WANT TO TUNE? Start with P, set I=0 and D=0. Increase P until the wheel
     * oscillates, then back off slightly. Add a tiny D if it still oscillates.
     */

    /** Drive motor P gain. Controls how aggressively the drive motor reaches target speed. */
    public static final double kPDrive = .21;
    /** Drive motor I gain. Usually 0 -- not needed for velocity control. */
    public static final double kIDrive = 0.0;
    /** Drive motor D gain. Usually 0 for drive motors. */
    public static final double kDDrive  = 0.0;

    /**
     * Rotation (steering) motor P gain. Controls how aggressively the wheel turns
     * to the target angle. 0.57 works well for our modules.
     *
     * WANT TO CHANGE? If wheels oscillate (jitter back and forth), decrease this.
     * If wheels are slow to reach the target angle, increase this.
     */
    public static final double kPRotation = 0.57;
    /** Rotation motor I gain. Usually 0. */
    public static final double kIRotation = 0.0;
    /**
     * Rotation motor D gain. Dampens oscillation in the steering.
     * A small value (0.005) smooths out the wheel's approach to the target angle.
     */
    public static final double kDRotation = 0.005;

    /**
     * How close (in radians) the wheel angle needs to be to the target before the
     * PID controller considers it "good enough." 0.01 rad is about 0.6 degrees.
     */
    public static final double kToleranceRotation = 0.01;


  }
}
