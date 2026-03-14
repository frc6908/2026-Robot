package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterMechanism extends SubsystemBase {
    private final SparkMax shooterSpark1;
    private final SparkMax shooterSpark2;
    private final SparkMax kickerSpark;
    private final SparkClosedLoopController kickerController;

    public ShooterMechanism(
        int shooterSparkPort1,
        int shooterSparkPort2,
        int kickerSparkPort
    ) {
        shooterSpark1 = new SparkMax(shooterSparkPort1, MotorType.kBrushless);
        shooterSpark2 = new SparkMax(shooterSparkPort2, MotorType.kBrushless);
        kickerSpark = new SparkMax(kickerSparkPort, MotorType.kBrushless);

        configureMotor(shooterSpark1, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureMotor(shooterSpark2, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureKicker();

        kickerController = kickerSpark.getClosedLoopController();
    }

    public void configureMotor(SparkMax spark, IdleMode idleMode, int currentLimit) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(idleMode)
            .smartCurrentLimit(currentLimit);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureKicker() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.currentLimit);
        config.closedLoop
            .p(ShooterConstants.kKickerP)
            .i(ShooterConstants.kKickerI)
            .d(ShooterConstants.kKickerD);
        config.closedLoop.feedForward.kV(ShooterConstants.kKickerFF);
        kickerSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIOSpark(double speed1, double speed2) {
        shooterSpark1.set(speed1);
        shooterSpark2.set(speed2);
        kickerController.setReference(ShooterConstants.kKickerTargetRPM, ControlType.kVelocity);
    }

    public void stopIOSpark() {
        shooterSpark1.stopMotor();
        shooterSpark2.stopMotor();
        kickerSpark.stopMotor();
    }

    @Override
    public void periodic() {}
}
