package frc.robot;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.BarConstants;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipFieldRelativity;
import frc.robot.commands.FlipFieldRelativity2;
import frc.robot.commands.Intake;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.IntakeDown;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.Outtake;
import frc.robot.commands.ResetNavX;
import frc.robot.commands.Shooter;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.HopperMechanism;
import frc.robot.subsystems.BarMechanism;
import frc.robot.subsystems.ShooterMechanism;
import frc.robot.commands.AlignToTag;
import frc.robot.subsystems.ClimbMechanism;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AlignAndShoot;
import frc.robot.commands.TrenchShootAuto;
import frc.robot.commands.InvertDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final SendableChooser<Command> autoChooser;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final SendableChooser<Command> autoChooser;
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();
  private final HopperMechanism m_hopperMech = new HopperMechanism(HopperConstants.hopperSparkPort);
  private final BarMechanism m_barMech = new BarMechanism(BarConstants.barSparkPort, BarConstants.barSpark2Port);
  private final ShooterMechanism m_shooterMech = new ShooterMechanism(ShooterConstants.shooterSparkPort1, ShooterConstants.shooterSparkPort2, ShooterConstants.kickerSparkPort);
  private final ClimbMechanism m_climbMech = new ClimbMechanism(ClimbConstants.climbSparkPort1);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Auto selection chooser
  SendableChooser<Command> autoChooser;
  private GenericEntry autoAlignEntry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


     //autoChooser = AutoBuilder.buildAutoChooser();
     //NamedCommands.registerCommand("AlgaeIntake", new IntakeAlgae(m_algaeMech));
     //NamedCommands.registerCommand("AlgaeOuttake", new OuttakeAlgae(m_algaeMech));
     //NamedCommands.registerCommand("ArmDown", new MoveArm(m_algaeMech, false));
     //NamedCommands.registerCommand("ArmUp", new MoveArm(m_algaeMech,true));
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain, 
      () -> m_driverController.getLeftY(), 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getRightX(),
      () -> m_driverController.getLeftTriggerAxis()
    ));

    // Configure the trigger bindings
    configureBindings();

    // Add auto options to the chooser
    //autoChooser.addOption("Algae Auto", "AlgaeAuto");
    //autoChooser.addOption("Custom Path Auto", "CustomPathAuto");

    NamedCommands.registerCommand("Intake", new Intake(m_hopperMech, m_barMech));
    NamedCommands.registerCommand("IntakeDown", new IntakeDown(m_barMech).withTimeout(1.0));
    NamedCommands.registerCommand("IntakeUp", new IntakeUp(m_barMech).withTimeout(1.0));
    NamedCommands.registerCommand("Outtake", new Outtake(m_hopperMech, m_barMech));
    NamedCommands.registerCommand("Shoot", new Shooter(m_shooterMech, m_hopperMech));
    NamedCommands.registerCommand("AutoShoot", new AutoShooter(m_shooterMech, m_drivetrain, m_hopperMech));
    NamedCommands.registerCommand("AlignToTag", new AlignToTag(m_drivetrain, () -> 0.0, () -> 0.0));
    NamedCommands.registerCommand("AlignAndShoot", new AlignAndShoot(m_drivetrain, m_shooterMech, m_hopperMech).withTimeout(3.0));
    //NamedCommands.registerCommand("Climb", new Climb(m_climbMech));

    autoChooser = AutoBuilder.buildAutoChooser("MobilityAuto");
    autoChooser.setDefaultOption("trenchShoot", new TrenchShootAuto(m_shooterMech, m_hopperMech));

    // Driver tab on Shuffleboard/Elastic
    // Camera stream is added by SwerveSubsystem at position (0,0) size (4,3)
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
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    // invert drivetrain controls
    m_driverController.back().onTrue(new InvertDrivetrain(m_drivetrain));

    // climb
    m_driverController.povDown().whileTrue(new ClimbDown(m_climbMech));
    m_driverController.povUp().whileTrue(new Climb(m_climbMech));

    // flip field relativity
    m_driverController.povLeft().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.povRight().whileTrue(new FlipFieldRelativity2(m_drivetrain));

    // reset navX heading
    m_driverController.y().whileTrue(new ResetNavX(m_drivetrain));

    // intake
    m_driverController.leftTrigger().whileTrue(new Intake(m_hopperMech, m_barMech));
    m_driverController.x().whileTrue(new Outtake(m_hopperMech, m_barMech));

    // intake bar
    m_driverController.leftBumper().whileTrue(new IntakeDown(m_barMech).withTimeout(1.0));
    m_driverController.a().whileTrue(new IntakeUp(m_barMech).withTimeout(1.0));

    // shooter
    m_driverController.rightTrigger().whileTrue(new Shooter(m_shooterMech, m_hopperMech));

    // align to tag
    m_driverController.rightBumper().whileTrue(new AlignToTag(
        m_drivetrain,
        () -> -m_driverController.getLeftY(), // Forward/Back
        () -> -m_driverController.getLeftX()  // Left/Right
    ));

    // auto shooter (distance-based)
    m_driverController.b().whileTrue(new AutoShooter(m_shooterMech, m_drivetrain, m_hopperMech));

    // === Operator Controller (same bindings, no joysticks) ===

    // invert drivetrain controls
    m_operatorController.povUp().onTrue(new InvertDrivetrain(m_drivetrain));

    // flip field relativity
    m_operatorController.povLeft().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_operatorController.povRight().whileTrue(new FlipFieldRelativity2(m_drivetrain));

    // reset navX heading
    m_operatorController.y().whileTrue(new ResetNavX(m_drivetrain));

    // intake
    m_operatorController.leftTrigger().whileTrue(new Intake(m_hopperMech, m_barMech));
    m_operatorController.x().whileTrue(new Outtake(m_hopperMech, m_barMech));

    // intake bar
    m_operatorController.leftBumper().whileTrue(new IntakeDown(m_barMech).withTimeout(1.0));
    m_operatorController.a().whileTrue(new IntakeUp(m_barMech).withTimeout(1.0));

    // shooter
    m_operatorController.rightTrigger().whileTrue(new Shooter(m_shooterMech, m_hopperMech));

    // align to tag
    m_operatorController.rightBumper().whileTrue(new AlignToTag(
        m_drivetrain,
        () -> 0.0, // No joystick input
        () -> 0.0
    ));

    // auto shooter (distance-based)
    m_operatorController.b().whileTrue(new AutoShooter(m_shooterMech, m_drivetrain, m_hopperMech));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public boolean isAutoAlignEnabled() {
    return autoAlignEntry.getBoolean(false);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
