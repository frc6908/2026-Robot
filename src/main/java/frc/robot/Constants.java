// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
//import edu.wpi.first.math.geometry.Translation2d;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {

    /* ================ */
    /* CONTROLLER PORTS */
    /* ================ */
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* ========= */
    /* Deadbands */
    /* ========= */
    public static final double xDeadband = 0.1;
    public static final double yDeadband = 0.1;
    public static final double rDeadband = 0.3;  
  }


  public static class IntakeConstants{
    public static final int ioSparkPort = 40;
    public static final double intakeSpeed = 0.75;
    public static final double outtakeSpeed = -0.75;

    public static final int currentLimit = 35;
  }

  public static class ClimbConstants {
    public static final int climbSparkPort1 = 45;
    public static final int currentLimit = 35;
  }


  public static class ShooterConstants {
        public static final int shooterSparkPort1 = 42;
        public static final int shooterSparkPort2 = 43;
        public static final int kickerSparkPort = 44;
        public static final int currentLimit = 35;

        // --- FRC 2026 REBUILT CONSTANTS ---
        
        // Tags located on the HUB (Central Structure)
        public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11}; 
        public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};

        // LOOKUP TABLE: Distance (Meters) -> Shooter Speed % (0.0 to 1.0)
        // The robot calculates the speed for any distance between these points.
        public static final InterpolatingDoubleTreeMap kDistanceToSpeedMap = new InterpolatingDoubleTreeMap();
        static {
            // MEASURE THESE ON THE FIELD!
            // Format: .put(Distance_Meters, Speed_Percent);
            
            kDistanceToSpeedMap.put(1.0, 0.35); // Close range (Fender shot)
            kDistanceToSpeedMap.put(2.5, 0.55); // Mid range
            kDistanceToSpeedMap.put(4.0, 0.75); // Long range
            kDistanceToSpeedMap.put(6.0, 0.95); // Cross-field
        }
    }

  public static class VisionConstants {
    public static final String kCameraName = "Global_Shutter_Camera";   

    // Transform from Robot Center to Camera Lens
    // X: Forward, Y: Left, Z: Up
    // Rotation: Roll, Pitch, Yaw
    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5), // ADJUST THIS BASED ON YOUR CAMERA POSITION (0.5m forward, 0m left, 0.5m up is an example)
        new Rotation3d(0, 0, 0)           // Example: Facing forward
    );
  }

  public static class DrivetrainConstants {
    // Swerve Kinematics, X forward/backward and Y is left/right
    public static final double wheelBase = Units.inchesToMeters(21); // distance between inside front wheels
    public static final double trackWidth = Units.inchesToMeters(21); // distance between side wheels
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // back right (+,-)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back left (-,-)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */
    
    // Given motor rotations, convert to meters traveled
    // (1 rev / Gear Ratio) * ((2 * PI * r) / (1 Rev)) = 
    // (2 * PI * r) / (Gear Ratio) = 
    public static final double drivePositionConversionFactor = 0.0472867872;

    // dx/dt
    // Given RPM, convert to m/s
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    // Given Motor Rotations, convert to Radians travelled
    // (1 rev / Gear Ratio) * ((2 * PI) RAD / (1 Rev))
    // (2 * PI) RAD / (Gear Ratio)
    public static final double rotationPositionConversionFactor = 0.29321531433;

    // Given RPM, convert to radians/seconds
    public static final double rotationVelocityConversionFactor = rotationPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // maximillian
    public static final double maxVelocity = 4; // m/s
    public static final double maxAcceleration = 7; // m/s^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/s
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/s^2
    // Teleop Max Speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // meters/sec
    public static final double kTeleDriveMaxAngularSpeed = 3; // rad/sec

    
    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

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

      public static final double kPDrive = .21;
      public static final double kIDrive = 0.0;
      public static final double kDDrive  = 0.0;
    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5, 0.0);

      public static final double kPRotation = 0.57; // working BL and FR p-value: .56
      public static final double kIRotation = 0.0;
      public static final double kDRotation = 0.005;
      public static final double kToleranceRotation = 0.01;

      
  }
}
