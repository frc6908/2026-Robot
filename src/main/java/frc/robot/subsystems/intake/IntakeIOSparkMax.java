package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;

/**
 * Hardware implementation of IntakeIO -- talks to the real SparkMax motor controller.
 *
 * This class creates and configures the SparkMax, reads real sensor values
 * (motor output, velocity, current), and sends speed commands to the actual motor.
 *
 * Only used on the real robot. In simulation, IntakeIOSim is used instead.
 */
public class IntakeIOSparkMax implements IntakeIO {

    /** The SparkMax motor controller for the intake roller. */
    private final SparkMax motor;

    /** Built-in encoder for reading motor velocity. */
    private final RelativeEncoder encoder;

    /** Tracks the last speed we set so we can log it. */
    private double appliedSpeed = 0.0;

    public IntakeIOSparkMax() {
        motor = new SparkMax(IntakeConstants.ioSparkPort, MotorType.kBrushless);

        // Configure with brake mode and current limit to protect the motor
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.currentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedSpeed = appliedSpeed;
        inputs.velocityRadPerSec = encoder.getVelocity() * (2.0 * Math.PI / 60.0); // RPM to rad/s
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setSpeed(double speed) {
        appliedSpeed = speed;
        motor.set(speed);
    }

    @Override
    public void stop() {
        appliedSpeed = 0.0;
        motor.stopMotor();
    }
}
