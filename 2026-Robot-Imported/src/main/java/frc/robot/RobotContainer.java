package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  /* ================= Subsystems ================= */
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();
  private final AlgaeMechanism m_algaeMech =
      new AlgaeMechanism(
          AlgaeConstants.ioSparkPort,
          AlgaeConstants.algaeArmSparkPort);

  /* ================= Controllers ================= */
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /* ================= Autonomous ================= */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    /* -------- PathPlanner Autos -------- */
    NamedCommands.registerCommand(
        "AlgaeIntake", new IntakeAlgae(m_algaeMech));
    NamedCommands.registerCommand(
        "AlgaeOuttake", new OuttakeAlgae(m_algaeMech));
    NamedCommands.registerCommand(
        "ArmDown", new MoveArm(m_algaeMech, false));
    NamedCommands.registerCommand(
        "ArmUp", new MoveArm(m_algaeMech, true));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /* -------- Default Drive Command -------- */
    m_drivetrain.setDefaultCommand(
        new SwerveJoystickCmd(
            m_drivetrain,
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX(),
            () -> m_driverController.getLeftTriggerAxis()));

    configureBindings();

    SmartDashboard.putBoolean("Use PathPlanner", true);
    SmartDashboard.putBoolean("Debug Swerve", false);
  }

  private void configureBindings() {

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Driver controls
    m_driverController.x()
        .whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.a()
        .whileTrue(new FlipFieldRelativity2(m_drivetrain));
    m_driverController.y()
        .whileTrue(new ResetNavX(m_drivetrain));

    // Operator controls
    m_operatorController.b()
        .whileTrue(new IntakeAlgae(m_algaeMech));
    m_operatorController.x()
        .whileTrue(new OuttakeAlgae(m_algaeMech));
    m_operatorController.a()
        .whileTrue(new MoveArm(m_algaeMech, false));
    m_operatorController.y()
        .whileTrue(new MoveArm(m_algaeMech, true));
    m_operatorController.rightBumper()
        .whileTrue(new ResetArmEncoder(m_algaeMech));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}