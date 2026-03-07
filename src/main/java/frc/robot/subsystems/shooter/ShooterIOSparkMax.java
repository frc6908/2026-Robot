package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ShooterConstants;

/**
 * Hardware implementation of ShooterIO -- talks to the real SparkMax motor controllers.
 *
 * Manages three motors: two shooter flywheel motors and one kicker motor.
 * Reads real sensor values and sends speed commands to the actual hardware.
 *
 * Only used on the real robot. In simulation, ShooterIOSim is used instead.
 */
public class ShooterIOSparkMax implements ShooterIO {

    private final SparkMax shooterSpark1;
    private final SparkMax shooterSpark2;
    private final SparkMax kickerSpark;

    private final RelativeEncoder encoder1;
    private final RelativeEncoder encoder2;
    private final RelativeEncoder kickerEncoder;

    private double shooter1AppliedSpeed = 0.0;
    private double shooter2AppliedSpeed = 0.0;
    private double kickerAppliedSpeed = 0.0;

    public ShooterIOSparkMax() {
        shooterSpark1 = new SparkMax(ShooterConstants.shooterSparkPort1, MotorType.kBrushless);
        shooterSpark2 = new SparkMax(ShooterConstants.shooterSparkPort2, MotorType.kBrushless);
        kickerSpark = new SparkMax(ShooterConstants.kickerSparkPort, MotorType.kBrushless);

        // Configure all three motors with brake mode and current limits
        configureMotor(shooterSpark1);
        configureMotor(shooterSpark2);
        configureMotor(kickerSpark);

        encoder1 = shooterSpark1.getEncoder();
        encoder2 = shooterSpark2.getEncoder();
        kickerEncoder = kickerSpark.getEncoder();
    }

    private void configureMotor(SparkMax spark) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.currentLimit);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooter1AppliedSpeed = shooter1AppliedSpeed;
        inputs.shooter1VelocityRadPerSec = encoder1.getVelocity() * (2.0 * Math.PI / 60.0);
        inputs.shooter1CurrentAmps = shooterSpark1.getOutputCurrent();

        inputs.shooter2AppliedSpeed = shooter2AppliedSpeed;
        inputs.shooter2VelocityRadPerSec = encoder2.getVelocity() * (2.0 * Math.PI / 60.0);
        inputs.shooter2CurrentAmps = shooterSpark2.getOutputCurrent();

        inputs.kickerAppliedSpeed = kickerAppliedSpeed;
        inputs.kickerVelocityRadPerSec = kickerEncoder.getVelocity() * (2.0 * Math.PI / 60.0);
        inputs.kickerCurrentAmps = kickerSpark.getOutputCurrent();
    }

    @Override
    public void setShooterSpeed(double speed1, double speed2) {
        shooter1AppliedSpeed = speed1;
        shooter2AppliedSpeed = speed2;
        shooterSpark1.set(speed1);
        shooterSpark2.set(speed2);
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerAppliedSpeed = speed;
        kickerSpark.set(speed);
    }

    @Override
    public void stop() {
        shooter1AppliedSpeed = 0.0;
        shooter2AppliedSpeed = 0.0;
        kickerAppliedSpeed = 0.0;
        shooterSpark1.stopMotor();
        shooterSpark2.stopMotor();
        kickerSpark.stopMotor();
    }
}
