package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.MobilityAuton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipFieldRelativity;
import frc.robot.commands.FlipFieldRelativity2;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveArm;
import frc.robot.commands.OuttakeAlgae;
import frc.robot.commands.ResetArmEncoder;
import frc.robot.commands.ResetNavX;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * RobotContainer is the "glue" of the robot. It:
 *   1. Creates all the subsystems (drivetrain, algae mechanism, etc.)
 *   2. Creates the Xbox controllers
 *   3. Connects buttons to commands ("when I press B, run the intake")
 *   4. Sets up the autonomous chooser on the dashboard
 *
 * Think of this as the control center where everything gets wired together.
 *
 * WANT TO CHANGE which button does what? → Edit configureBindings() below.
 * WANT TO ADD a new subsystem? → Create it as a field (like m_drivetrain),
 *   then create commands for it and bind them in configureBindings().
 * WANT TO ADD a new autonomous routine? → Register commands with NamedCommands
 *   in the constructor, then create the path in PathPlanner.
 */
public class RobotContainer {
  // --- Subsystems ---
  // Subsystems are the major hardware groups on the robot.
  // Each one manages its own motors, sensors, and logic.
  private final SendableChooser<Command> autoChooser;

  // NOTE: ExampleSubsystem is WPILib boilerplate -- it doesn't control any real
  // hardware. It's safe to ignore. The real subsystems are below.
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // --- The actual subsystems that control robot hardware ---
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();
  private final AlgaeMechanism m_algaeMech = new AlgaeMechanism(AlgaeConstants.ioSparkPort, AlgaeConstants.algaeArmSparkPort);

  // --- Controllers ---
  // We use two Xbox controllers: one for the driver (movement) and one for the
  // operator (mechanisms like the arm and intake).
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // --- Auto Selection ---
  // SendableChooser creates a dropdown menu on the dashboard so you can pick
  // which autonomous routine to run before the match starts.
  SendableChooser<String> autoChooser = new SendableChooser<>();
  SendableChooser<String> AllianceChooser = new SendableChooser<>();

  /**
   * Sets everything up when the robot boots. This only runs once.
   */
  public RobotContainer() {

    // --- PathPlanner Autonomous Setup ---
    // AutoBuilder creates a dropdown of all the auto routines we've made in PathPlanner.
    // NamedCommands lets us use our commands (like IntakeAlgae) inside PathPlanner paths
    // by giving them string names that PathPlanner can reference.
     autoChooser = AutoBuilder.buildAutoChooser();
     NamedCommands.registerCommand("AlgaeIntake", new IntakeAlgae(m_algaeMech));
     NamedCommands.registerCommand("AlgaeOuttake", new OuttakeAlgae(m_algaeMech));
     NamedCommands.registerCommand("ArmDown", new MoveArm(m_algaeMech, false));
     NamedCommands.registerCommand("ArmUp", new MoveArm(m_algaeMech,true));
     SmartDashboard.putData("AutoChooser", autoChooser);


    // --- Default Command ---
    // A "default command" runs whenever no other command is using that subsystem.
    // For the drivetrain, the default command is joystick control -- so as long as
    // no auto path or other command is running, the driver has control.
    //
    // The () -> syntax is a "lambda" -- it's a way to pass a function as a value.
    // Instead of reading the joystick once and passing a fixed number, we pass
    // a function that reads the joystick fresh every time it's called (every 20ms).
    //
    // WANT TO CHANGE which joystick axis controls what?
    //   Swap the getLeftY/getLeftX/getRightX calls below. For example, to use
    //   the right stick for movement, change getLeftY to getRightY, etc.
    //
    // WANT TO USE A DIFFERENT CONTROLLER (like a PS4 controller)?
    //   Replace CommandXboxController with CommandPS4Controller in the fields above,
    //   then update the button names (e.g., .cross() instead of .a()).
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain,
      () -> m_driverController.getLeftY(),          // forward/backward (Y axis)
      () -> m_driverController.getLeftX(),          // left/right strafe (X axis)
      () -> m_driverController.getRightX(),         // rotation
      () -> m_driverController.getLeftTriggerAxis() // speed slider (trigger)
    ));

    // Wire up all the button-to-command mappings
    configureBindings();

    // Add auto options to the chooser dropdown on the dashboard
    autoChooser.setDefaultOption("Mobility Auto", "MobilityAuto");
    autoChooser.addOption("Algae Auto", "AlgaeAuto");
    autoChooser.addOption("Custom Path Auto", "CustomPathAuto");

    // Send data to the SmartDashboard so we can see/change things from the laptop
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
  }

  /**
   * Button bindings -- this is where we say "when button X is pressed, do Y."
   *
   * "whileTrue" means the command runs as long as the button is held down,
   * and stops when you let go. This is good for things like intake motors
   * that should only run while you're pressing the button.
   *
   * WANT TO CHANGE the button layout? Change the button method calls below.
   *   Available buttons: a(), b(), x(), y(), leftBumper(), rightBumper(),
   *   leftTrigger(), rightTrigger(), start(), back()
   *
   * WANT TO ADD a new button binding?
   *   Follow this pattern:
   *     m_driverController.BUTTON().whileTrue(new YourCommand(m_yourSubsystem));
   *
   * WANT A COMMAND TO RUN ONCE on button press instead of while held?
   *   Use .onTrue() instead of .whileTrue()
   */
  private void configureBindings() {
    // NOTE: This is WPILib boilerplate -- ExampleCommand and ExampleSubsystem don't
    // do anything real. exampleCondition() always returns false, so this trigger
    // never fires. It's here as a reference for how to use Trigger with a custom
    // condition (instead of a button). You can safely ignore this.
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // --- Driver Controller (port 0) ---
    // X button: switch to field-relative driving (robot moves relative to the field,
    //           so "forward" always means toward the other end of the field)
    m_driverController.x().whileTrue(new FlipFieldRelativity(m_drivetrain));

    // A button: switch to robot-relative driving (robot moves relative to itself,
    //           so "forward" always means wherever the front of the robot is pointing)
    m_driverController.a().whileTrue(new FlipFieldRelativity2(m_drivetrain));

    // Y button: reset the NavX gyro heading to 0 degrees (use this when the robot's
    //           "forward" direction gets out of sync with the real forward direction)
    m_driverController.y().whileTrue(new ResetNavX(m_drivetrain));

    // --- Operator Controller (port 1) ---
    // B button: run the intake rollers (suck algae in)
    m_operatorController.b().whileTrue(new IntakeAlgae(m_algaeMech));

    // X button: run the outtake rollers (spit algae out)
    m_operatorController.x().whileTrue(new OuttakeAlgae(m_algaeMech));

    // A button: move the arm down
    m_operatorController.a().whileTrue(new MoveArm(m_algaeMech, false));

    // Y button: move the arm up
    m_operatorController.y().whileTrue(new MoveArm(m_algaeMech, true));

    // Right bumper: reset the arm encoder to zero (recalibrate the arm position)
    m_operatorController.rightBumper().whileTrue(new ResetArmEncoder(m_algaeMech));
  }

  /**
   * Returns whichever autonomous command was selected on the dashboard.
   * This gets called by Robot.java at the start of autonomous mode.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
