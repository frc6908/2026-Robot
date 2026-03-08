package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipFieldRelativity;
import frc.robot.commands.FlipFieldRelativity2;
import frc.robot.commands.Intake;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.Outtake;
import frc.robot.commands.ResetNavX;
import frc.robot.commands.Shooter;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AlignAndShoot;

import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeMechanism;

import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterMechanism;

import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbMechanism;

import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.GyroIOSim;
import frc.robot.subsystems.swerve.VisionIO;
import frc.robot.subsystems.swerve.VisionIOLimelight;
import frc.robot.subsystems.swerve.VisionIOSim;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOSparkMax;
import frc.robot.subsystems.swerve.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import frc.robot.subsystems.ExampleSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.system.plant.DCMotor;


/**
 * This is where everything gets wired together. Think of it as the robot's "control room"
 * where we:
 *   1. Create all subsystems (drivetrain, intake, shooter, climb)
 *   2. Bind controller buttons to commands ("when B is pressed, run the intake")
 *   3. Register named commands for PathPlanner autonomous routines
 *   4. Set up the autonomous mode chooser on the dashboard
 *
 * The IO pattern is used here to pick the right implementation for each subsystem:
 *   - On the real robot (Robot.isReal()), we use hardware IO classes (SparkMax, NavX, Limelight)
 *   - In simulation (!Robot.isReal()), we use sim IO classes (physics simulation)
 *
 * WANT TO ADD A NEW BUTTON BINDING? Go to configureBindings().
 * WANT TO ADD A NEW SUBSYSTEM? Create the subsystem object here and pass it to commands.
 * WANT TO ADD A NEW AUTO ROUTINE? Register named commands below and create the .auto file
 * in PathPlanner.
 */
public class RobotContainer {

  // ========================
  // SUBSYSTEM INSTANTIATION
  // ========================

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The swerve drivetrain -- controls all 4 swerve modules and handles odometry/vision. */
  private final SwerveSubsystem m_drivetrain;

  /** The intake mechanism -- single roller motor for sucking in or spitting out game pieces. */
  private final IntakeMechanism m_intakeMech;

  /** The shooter mechanism -- dual flywheel motors + kicker for launching game pieces. */
  private final ShooterMechanism m_shooterMech;

  /** The climb mechanism -- single motor for lifting the robot onto the chain. */
  private final ClimbMechanism m_climbMech;

  // ========================
  // CONTROLLER SETUP
  // ========================

  /**
   * Driver controller (port 0): Controls robot movement, shooting, and alignment.
   * Left stick = drive, Right stick = rotate, Triggers = speed reduction.
   */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * Operator controller (port 1): Controls mechanisms (intake, outtake, climb).
   * Separate from the driver so each person can focus on their role.
   */
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // ========================
  // DASHBOARD WIDGETS
  // ========================

  /** Dropdown on the dashboard that lets the drive team choose which auto routine to run. */
  SendableChooser<Command> autoChooser;

  /** Toggle button on the dashboard for enabling/disabling automatic alignment. */
  private GenericEntry autoAlignEntry;

  /**
   * Constructor -- runs once when the robot boots up.
   * Sets up the default drive command, binds buttons, registers auto commands,
   * and configures the dashboard.
   */
  public RobotContainer() {

    // ========================
    // IO PATTERN: PICK REAL OR SIM
    // ========================
    // Robot.isReal() returns true on the actual robot, false in simulation.
    // We use this to pick the right IO implementation for each subsystem.

    if (Robot.isReal()) {
      // --- REAL ROBOT ---
      // Use hardware IO classes that talk to actual SparkMax motors, NavX, and Limelight

      m_intakeMech = new IntakeMechanism(new IntakeIOSparkMax());
      m_shooterMech = new ShooterMechanism(new ShooterIOSparkMax());
      m_climbMech = new ClimbMechanism(new ClimbIOSparkMax());

      m_drivetrain = new SwerveSubsystem(
          new GyroIONavX(),
          new VisionIOLimelight(),
          new SwerveModuleIOSparkMax(
              DrivetrainConstants.kFLDrive, DrivetrainConstants.kFLRotate,
              DrivetrainConstants.kFLCanCoder, DrivetrainConstants.kFLOffsetRad,
              DrivetrainConstants.fLIsInverted),
          new SwerveModuleIOSparkMax(
              DrivetrainConstants.kFRDrive, DrivetrainConstants.kFRRotate,
              DrivetrainConstants.kFRCanCoder, DrivetrainConstants.kFROffsetRad,
              DrivetrainConstants.fRIsInverted),
          new SwerveModuleIOSparkMax(
              DrivetrainConstants.kBLDrive, DrivetrainConstants.kBLRotate,
              DrivetrainConstants.kBLCanCoder, DrivetrainConstants.kBLOffsetRad,
              DrivetrainConstants.bLIsInverted),
          new SwerveModuleIOSparkMax(
              DrivetrainConstants.kBRDrive, DrivetrainConstants.kBRRotate,
              DrivetrainConstants.kBRCanCoder, DrivetrainConstants.kBROffsetRad,
              DrivetrainConstants.bRIsInverted)
      );
    } else {
      // --- SIMULATION ---
      // Use sim IO classes backed by physics simulations (maple-sim for swerve)

      m_intakeMech = new IntakeMechanism(new IntakeIOSim());
      m_shooterMech = new ShooterMechanism(new ShooterIOSim());
      m_climbMech = new ClimbMechanism(new ClimbIOSim());

      // Create the maple-sim drivetrain simulation with our robot's physical specs.
      // COTS.ofMAXSwerve creates a pre-configured swerve module matching REV MAXSwerve hardware.
      // Parameters: drive motor, steer motor, drive gear ratio, wheel COF (tread grip)
      DriveTrainSimulationConfig simConfig = DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(COTS.ofMAXSwerve(
              DCMotor.getNEO(1),       // Drive motor: NEO
              DCMotor.getNEO(1),       // Steer motor: NEO
              6.75,                     // Drive gear ratio
              1                         // Wheel COF tier (1 = standard tread)
          ))
          .withTrackLengthTrackWidth(
              edu.wpi.first.units.Units.Meters.of(DrivetrainConstants.wheelBase),
              edu.wpi.first.units.Units.Meters.of(DrivetrainConstants.trackWidth)
          )
          .withBumperSize(
              edu.wpi.first.units.Units.Meters.of(DrivetrainConstants.wheelBase + 0.1),
              edu.wpi.first.units.Units.Meters.of(DrivetrainConstants.trackWidth + 0.1)
          );

      SwerveDriveSimulation driveSim = new SwerveDriveSimulation(simConfig, new Pose2d());
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

      m_drivetrain = new SwerveSubsystem(
          new GyroIOSim(driveSim.getGyroSimulation()),
          new VisionIOSim(),
          new SwerveModuleIOSim(driveSim.getModules()[0]),
          new SwerveModuleIOSim(driveSim.getModules()[1]),
          new SwerveModuleIOSim(driveSim.getModules()[2]),
          new SwerveModuleIOSim(driveSim.getModules()[3])
      );
    }

    // Set the default command for the drivetrain: joystick driving.
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain,
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX(),
      () -> m_driverController.getRightX(),
      () -> m_driverController.getLeftTriggerAxis()
    ));

    // Configure the trigger bindings (buttons -> commands)
    configureBindings();

    // ========================
    // PATHPLANNER NAMED COMMANDS
    // ========================
    NamedCommands.registerCommand("Intake", new Intake(m_intakeMech));
    NamedCommands.registerCommand("Outtake", new Outtake(m_intakeMech));
    NamedCommands.registerCommand("Shoot", new Shooter(m_shooterMech));
    NamedCommands.registerCommand("AutoShoot", new AutoShooter(m_shooterMech, m_drivetrain));
    NamedCommands.registerCommand("AlignToTag", new AlignToTag(m_drivetrain, () -> 0.0, () -> 0.0));
    NamedCommands.registerCommand("AlignAndShoot", new AlignAndShoot(m_drivetrain, m_shooterMech).withTimeout(3.0));
    NamedCommands.registerCommand("Climb", new Climb(m_climbMech));

    // Build the auto chooser dropdown. "MobilityAuto" is the default selection.
    autoChooser = AutoBuilder.buildAutoChooser("MobilityAuto");

    // ========================
    // SHUFFLEBOARD / DASHBOARD
    // ========================

    Shuffleboard.getTab("Driver")
        .add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(4, 0)
        .withSize(3, 1);

    autoAlignEntry = Shuffleboard.getTab("Driver")
        .add("Auto Alignment", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(4, 1)
        .withSize(3, 1)
        .getEntry();

    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
  }

  /**
   * Configures button-to-command mappings for both controllers.
   *
   * DRIVER CONTROLLER (port 0):
   *   X button     = Enable field-relative driving
   *   A button     = Disable field-relative driving (robot-relative)
   *   Y button     = Reset NavX gyro heading (re-zeroes "forward")
   *   B button     = Run shooter at constant speed
   *   Left Bumper  = Auto-shooter (distance-based speed)
   *   Right Bumper = Align to AprilTag (auto-rotation)
   *
   * OPERATOR CONTROLLER (port 1):
   *   B button     = Run intake (suck in game piece)
   *   X button     = Run outtake (spit out game piece)
   *   Left Bumper  = Climb up
   *   Right Bumper = Climb down
   */
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // --- DRIVER CONTROLLER BINDINGS ---
    m_driverController.x().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.a().whileTrue(new FlipFieldRelativity2(m_drivetrain));
    m_driverController.y().whileTrue(new ResetNavX(m_drivetrain));
    m_driverController.b().whileTrue(new Shooter(m_shooterMech));
    m_driverController.leftBumper().whileTrue(new AutoShooter(m_shooterMech, m_drivetrain));
    m_driverController.rightBumper().whileTrue(new AlignToTag(
        m_drivetrain,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX()
    ));

    // --- OPERATOR CONTROLLER BINDINGS ---
    m_operatorController.b().whileTrue(new Intake(m_intakeMech));
    m_operatorController.x().whileTrue(new Outtake(m_intakeMech));
    m_operatorController.leftBumper().whileTrue(new Climb(m_climbMech));
    m_operatorController.rightBumper().whileTrue(new ClimbDown(m_climbMech));
  }

  public boolean isAutoAlignEnabled() {
    return autoAlignEntry.getBoolean(false);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
