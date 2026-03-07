package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;

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
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.commands.AlignToTag;
import frc.robot.subsystems.ClimbMechanism;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AlignAndShoot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This is where everything gets wired together. Think of it as the robot's "control room"
 * where we:
 *   1. Create all subsystems (drivetrain, intake, shooter, climb)
 *   2. Bind controller buttons to commands ("when B is pressed, run the intake")
 *   3. Register named commands for PathPlanner autonomous routines
 *   4. Set up the autonomous mode chooser on the dashboard
 *
 * The pattern is: Subsystems own the hardware. Commands use subsystems to do things.
 * This class connects buttons to commands and commands to subsystems.
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
  // Each subsystem represents a physical mechanism on the robot.
  // We create them here so they exist for the entire match.

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The swerve drivetrain -- controls all 4 swerve modules and handles odometry/vision. */
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();

  /** The intake mechanism -- single roller motor for sucking in or spitting out game pieces. */
  private final IntakeMechanism m_intakeMech = new IntakeMechanism(IntakeConstants.ioSparkPort);

  /** The shooter mechanism -- dual flywheel motors + kicker for launching game pieces. */
  private final ShooterMechanism m_shooterMech = new ShooterMechanism(ShooterConstants.shooterSparkPort1, ShooterConstants.shooterSparkPort2, ShooterConstants.kickerSparkPort);

  /** The climb mechanism -- single motor for lifting the robot onto the chain. */
  private final ClimbMechanism m_climbMech = new ClimbMechanism(ClimbConstants.climbSparkPort1);

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

    // Set the default command for the drivetrain: joystick driving.
    // A "default command" runs whenever no other command is using the drivetrain.
    // This means the robot is always drivable unless another command takes over.
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain,
      () -> m_driverController.getLeftY(),      // Forward/backward (Y-axis is inverted on controllers)
      () -> m_driverController.getLeftX(),      // Left/right strafe
      () -> m_driverController.getRightX(),     // Rotation
      () -> m_driverController.getLeftTriggerAxis() // Speed slider (pull trigger to slow down)
    ));

    // Configure the trigger bindings (buttons -> commands)
    configureBindings();

    // ========================
    // PATHPLANNER NAMED COMMANDS
    // ========================
    // Named commands allow PathPlanner autonomous routines to trigger robot actions.
    // In a .auto file, you can add a "NamedCommand" event with one of these names
    // and the corresponding command will run at that point in the path.
    NamedCommands.registerCommand("Intake", new Intake(m_intakeMech));
    NamedCommands.registerCommand("Outtake", new Outtake(m_intakeMech));
    NamedCommands.registerCommand("Shoot", new Shooter(m_shooterMech));
    NamedCommands.registerCommand("AutoShoot", new AutoShooter(m_shooterMech, m_drivetrain));
    NamedCommands.registerCommand("AlignToTag", new AlignToTag(m_drivetrain, () -> 0.0, () -> 0.0));
    NamedCommands.registerCommand("AlignAndShoot", new AlignAndShoot(m_drivetrain, m_shooterMech).withTimeout(3.0));
    NamedCommands.registerCommand("Climb", new Climb(m_climbMech));

    // Build the auto chooser dropdown. "MobilityAuto" is the default selection.
    // PathPlanner scans the deploy/pathplanner/autos/ folder for .auto files.
    autoChooser = AutoBuilder.buildAutoChooser("MobilityAuto");

    // ========================
    // SHUFFLEBOARD / DASHBOARD
    // ========================
    // Add widgets to the "Driver" tab so the drive team can see and control things.

    // Auto chooser dropdown -- lets the drive team pick which autonomous to run
    Shuffleboard.getTab("Driver")
        .add("Auto Chooser", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(4, 0)
        .withSize(3, 1);

    // Auto alignment toggle -- a button on the dashboard to enable/disable auto-alignment
    autoAlignEntry = Shuffleboard.getTab("Driver")
        .add("Auto Alignment", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(4, 1)
        .withSize(3, 1)
        .getEntry();

    // Debug flags for development
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
  }

  /**
   * Configures button-to-command mappings for both controllers.
   *
   * How button bindings work:
   *   - .whileTrue(command) = runs the command as long as the button is held down,
   *     stops when released
   *   - .onTrue(command) = runs the command once when the button is first pressed
   *   - .toggleOnTrue(command) = starts the command on first press, stops on second press
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
    // Schedule ExampleCommand when exampleCondition changes to true
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // --- DRIVER CONTROLLER BINDINGS ---

    // Field relativity controls: X = field-relative ON, A = field-relative OFF
    m_driverController.x().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.a().whileTrue(new FlipFieldRelativity2(m_drivetrain));

    // Reset the NavX gyro heading -- makes the robot's current facing direction "forward"
    m_driverController.y().whileTrue(new ResetNavX(m_drivetrain));

    // Shooter: B = constant speed, Left Bumper = auto-speed based on distance
    m_driverController.b().whileTrue(new Shooter(m_shooterMech));
    m_driverController.leftBumper().whileTrue(new AutoShooter(m_shooterMech, m_drivetrain));

    // Align to AprilTag: auto-rotates to face the target while still allowing manual driving
    m_driverController.rightBumper().whileTrue(new AlignToTag(
        m_drivetrain,
        () -> -m_driverController.getLeftY(), // Forward/Back (negated because Y-axis is inverted)
        () -> -m_driverController.getLeftX()  // Left/Right (negated for same reason)
    ));

    // --- OPERATOR CONTROLLER BINDINGS ---

    // Intake/outtake game pieces
    m_operatorController.b().whileTrue(new Intake(m_intakeMech));
    m_operatorController.x().whileTrue(new Outtake(m_intakeMech));

    // Climbing controls
    m_operatorController.leftBumper().whileTrue(new Climb(m_climbMech));
    m_operatorController.rightBumper().whileTrue(new ClimbDown(m_climbMech));
  }

  /**
   * Returns whether the auto-alignment toggle on the dashboard is enabled.
   * Can be checked by other parts of the code to conditionally enable alignment.
   *
   * @return true if auto-alignment is enabled on the dashboard
   */
  public boolean isAutoAlignEnabled() {
    return autoAlignEntry.getBoolean(false);
  }

  /**
   * Returns the autonomous command selected on the dashboard dropdown.
   * Called by Robot.java when autonomous mode starts.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
