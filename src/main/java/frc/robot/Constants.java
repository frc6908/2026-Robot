// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * All robot-wide numbers live here. Think of this as the robot's "settings" file.
 *
 * We keep everything in one place so if you need to change a motor ID, PID value,
 * or speed limit, you only change it here -- not scattered across 10 different files.
 *
 * Each inner class groups related constants together (controller stuff, algae stuff,
 * drivetrain stuff, etc.).
 *
 * THIS IS THE FIRST FILE TO LOOK AT when you want to change robot behavior.
 * Most tweaks (speed, sensitivity, motor IDs, PID tuning) happen here.
 */
public final class Constants {

  /**
   * Settings related to the driver/operator Xbox controllers.
   */
  public static class OperatorConstants {

    /* ================ */
    /* CONTROLLER PORTS */
    /* ================ */
    // USB port numbers in the Driver Station software.
    // Port 0 = the first controller plugged in (driver), Port 1 = second (operator).
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* ========= */
    /* Deadbands */
    /* ========= */
    // Joysticks are never perfectly centered -- they might read 0.02 even when
    // you're not touching them. A "deadband" ignores small values like that
    // so the robot doesn't drift when nobody is touching the sticks.
    // Any joystick reading below the deadband value gets treated as 0.
    //
    // WANT TO CHANGE how sensitive the joystick feels?
    //   - Increase the deadband → more of the stick's range is ignored (less twitchy)
    //   - Decrease the deadband → the robot responds to smaller stick movements (more responsive)
    //   - If the robot drifts when you're not touching the sticks, increase these values
    public static final double xDeadband = 0.1;   // left/right movement
    public static final double yDeadband = 0.1;   // forward/backward movement
    public static final double rDeadband = 0.3;   // rotation (higher because rotation is more sensitive)
  }

  /**
   * Settings for the algae intake/outtake mechanism (the arm + rollers).
   */
  public static class AlgaeConstants {
    // CAN IDs for the two motors on the algae mechanism
    // (CAN is the communication bus that connects all the motor controllers)
    public static final int ioSparkPort = 40;         // intake/outtake roller motor
    public static final int algaeArmSparkPort = 41;   // arm pivot motor

    // The arm encoder plugs into the RoboRIO's DIO (Digital Input/Output) ports.
    // A quadrature encoder uses two channels (A and B) to track position and direction.
    public static final int algaeArmEncoderChannelA = 0;
    public static final int algaeArmEncoderChannelB = 1;

    // Motor speeds as a fraction of full power (1.0 = 100%).
    // Positive = intake (suck in), negative = outtake (spit out).
    //
    // WANT TO CHANGE the intake/outtake or arm speed?
    //   - Increase toward 1.0 for faster, decrease toward 0.0 for slower
    //   - Outtake is negative because the motor needs to spin the opposite direction
    //   - Keep algaeArmSpeed low (under 0.5) so the arm doesn't move dangerously fast
    public static final double intakeSpeed = 0.75;
    public static final double outtakeSpeed = -0.75;
    public static final double algaeArmSpeed = 0.2;   // arm moves slower for safety/control

    // Maximum current (in amps) the motor controllers will allow.
    // This protects motors and gears from burning out if the mechanism gets stuck.
    //
    // WANT TO CHANGE the current limit?
    //   - Lower = safer for motors but mechanism might stall under load
    //   - Higher = more power available but higher risk of burning out motors/gears
    //   - NEO motors can handle up to 80A in short bursts, but 30-40A is a safe continuous limit
    public static final int currentLimit = 35;

    // Arm rotation limits (in number of full rotations of the encoder).
    // These define how far up and down the arm is allowed to go.
    //
    // WANT TO CHANGE how far the arm can move?
    //   - Adjust upperLimitRotation and lowerLimitRotation
    //   - NOTE: the soft stop code in MoveArm.java is currently commented out,
    //     so these limits aren't enforced yet. See MoveArm.java to enable them.
    public static final double upperLimitRotation = 0.5;
    public static final double lowerLimitRotation = 0;
    // How close to the limit (in rotations) before the arm starts slowing down.
    // This prevents the arm from slamming into its hard stops.
    public static final double softStopDistance = 0.2;
  }

  /**
   * Settings for the swerve drivetrain -- this is the big one.
   *
   * Swerve drive means each of the 4 wheels can independently spin (drive)
   * AND rotate (steer). This lets the robot move in any direction while
   * facing any direction -- like a hockey player skating sideways.
   */
  public static class DrivetrainConstants {

    // Physical dimensions of the drivetrain (center-to-center between wheels).
    // We need these so the math knows where each wheel is relative to the center.
    public static final double wheelBase = Units.inchesToMeters(21);  // front-to-back distance
    public static final double trackWidth = Units.inchesToMeters(21); // left-to-right distance
    public static final double wheelDiameter = Units.inchesToMeters(4.0);

    // SwerveDriveKinematics tells WPILib where each wheel is relative to the robot's center.
    // It uses this to figure out how fast and in what direction each wheel needs to go
    // when you want the whole robot to move a certain way.
    // Each Translation2d is (x, y) in meters from the center of the robot.
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),    // front right (+x, +y)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),   // back right (+x, -y)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),   // front left (-x, +y)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)   // back left (-x, -y)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */
    // Motors count in "rotations", but we want to think in meters and radians.
    // These conversion factors translate between the two.

    // How many meters the robot travels per one rotation of the drive motor.
    // Accounts for the gear ratio between the motor and the wheel.
    // Formula: (1 / gear_ratio) * (2 * PI * wheel_radius)
    public static final double drivePositionConversionFactor = 0.0472867872;

    // How many meters/second per RPM of the drive motor (just position factor / 60 seconds).
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    // How many radians the wheel turns per one rotation of the steering motor.
    // Formula: (1 / gear_ratio) * (2 * PI)
    public static final double rotationPositionConversionFactor = 0.29321531433;

    // How many radians/second per RPM of the steering motor.
    public static final double rotationVelocityConversionFactor = rotationPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // The fastest the robot is physically capable of going
    //
    // WANT TO CHANGE the robot's max speed?
    //   - maxVelocity: the absolute top speed used during driving. Lowering this
    //     makes the robot slower overall. This is what gets multiplied by your
    //     joystick input in SwerveJoystickCmd.
    //   - maxAcceleration: how quickly the robot can speed up/slow down.
    //     Lower = smoother but sluggish. Higher = snappier but more jerky.
    //   - maxAngularVelocity/maxAngularAcceleration: same idea but for spinning.
    public static final double maxVelocity = 4; // meters per second
    public static final double maxAcceleration = 7; // meters per second squared
    public static final double maxAngularVelocity = 2 * Math.PI; // one full rotation per second
    public static final double maxAngularAcceleration = 4 * Math.PI; // radians per second squared

    // Teleop speed limits -- intentionally slower than the physical max
    // so the robot is easier to control during driver-controlled periods.
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // ~1.875 meters/sec
    public static final double kTeleDriveMaxAngularSpeed = 3;   // radians/sec


    /* ============== */
    /* SWERVE MODULES */
    /* ============== */
    // Each swerve module has 3 things on the CAN bus:
    //   1. A drive motor (SparkMax) -- spins the wheel forward/backward
    //   2. A rotation motor (SparkMax) -- steers the wheel left/right
    //   3. A CANcoder -- an absolute encoder that always knows which way the wheel is pointing
    //
    // The "offset" is a calibration value. When we first set up each module, the
    // CANcoder's zero point probably doesn't line up with "straight ahead."
    // The offset corrects for that so 0 radians = wheel pointing forward.
    //
    // SWAPPED A MOTOR or changed wiring? Update the CAN IDs here.
    //   - Match each ID to the number configured on the physical motor controller.
    //   - You can check/set CAN IDs using the REV Hardware Client or Phoenix Tuner.
    //
    // WHEELS NOT POINTING STRAIGHT when the robot boots?
    //   - You need to recalibrate the offset values (kFLOffsetRad, etc.).
    //   - Point each wheel straight forward, read the raw CANcoder value, and
    //     put that value as the offset (multiply by 2*PI to convert to radians).

    // Front Left Module
    public static final int kFLDrive = 4;
    public static final int kFLRotate = 7;
    public static final int kFLCanCoder = 14;
    public static final double kFLOffsetRad = 0.042969 * 2 * Math.PI;
    public static final boolean fLIsInverted = true;

    // Front Right Module
    public static final int kFRDrive = 5;
    public static final int kFRRotate = 3;
    public static final int kFRCanCoder = 13;
    public static final double kFROffsetRad = 0.028564 * 2 * Math.PI;
    public static final boolean fRIsInverted = true;

    // Back Left Module
    public static final int kBLDrive = 6;
    public static final int kBLRotate = 1;
    public static final int kBLCanCoder = 12;
    public static final double kBLOffsetRad = 0.027466 * 2 * Math.PI;
    public static final boolean bLIsInverted = true;

    // Back Right Module
    public static final int kBRDrive = 8;
    public static final int kBRRotate = 2;
    public static final int kBRCanCoder = 11;
    public static final double kBROffsetRad = 0.340698 * 2 * Math.PI;
    public static final boolean bRIsInverted = true;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */
    // PID stands for Proportional-Integral-Derivative. It's a control algorithm
    // that tries to smoothly reach a target value (like a desired speed or angle).
    //   P = how hard to correct based on current error (most important one)
    //   I = corrects for small errors that build up over time (usually 0 for us)
    //   D = slows down as we get close to the target to prevent overshooting
    //
    // WANT TO TUNE PID? (if wheels are oscillating, overshooting, or sluggish)
    //   - Wheels wiggling/oscillating back and forth? → decrease kPRotation
    //   - Wheels too slow to reach the target angle? → increase kPRotation
    //   - Wheels overshoot then correct? → increase kDRotation slightly
    //   - Robot drives at the wrong speed? → adjust kPDrive
    //   - Start by changing P in small increments (0.05-0.1) and test each time

    // PID values for the drive motor (controls wheel speed)
      public static final double kPDrive = .21;
      public static final double kIDrive = 0.0;
      public static final double kDDrive  = 0.0;

    // PID values for the rotation/steering motor (controls wheel direction)
      public static final double kPRotation = 0.57;
      public static final double kIRotation = 0.0;
      public static final double kDRotation = 0.005;
      // How close (in radians) the wheel angle needs to be to the target to be "good enough"
      public static final double kToleranceRotation = 0.01;


  }
}
