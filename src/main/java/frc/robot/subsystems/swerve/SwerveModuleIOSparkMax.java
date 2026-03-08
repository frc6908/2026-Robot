package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.DrivetrainConstants;

/**
 * Hardware implementation of SwerveModuleIO -- talks to real SparkMax motor
 * controllers and a CANcoder absolute encoder.
 *
 * Each swerve module has:
 *   - Drive motor (SparkMax + NEO) for wheel speed
 *   - Rotation motor (SparkMax + NEO) for wheel angle
 *   - CANcoder for absolute wheel angle (survives power cycles)
 *
 * Only used on the real robot. In simulation, SwerveModuleIOSim is used instead.
 */
public class SwerveModuleIOSparkMax implements SwerveModuleIO {

    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;
    private final CANcoder canCoder;
    private final double canCoderOffsetRadians;

    private double driveAppliedVolts = 0.0;
    private double rotationAppliedVolts = 0.0;

    /**
     * Creates a new SwerveModuleIOSparkMax with the given hardware IDs.
     *
     * @param driveMotorID         CAN ID of the drive motor SparkMax
     * @param rotationMotorID      CAN ID of the rotation motor SparkMax
     * @param canCoderID           CAN ID of the CANcoder absolute encoder
     * @param canCoderOffsetRadians calibration offset for the CANcoder (in radians)
     * @param isDriveInverted      whether the drive motor direction is inverted
     */
    public SwerveModuleIOSparkMax(
        int driveMotorID,
        int rotationMotorID,
        int canCoderID,
        double canCoderOffsetRadians,
        boolean isDriveInverted
    ) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

        // Configure drive motor with conversion factors for meters
        configureMotor(driveMotor, isDriveInverted,
            DrivetrainConstants.drivePositionConversionFactor,
            DrivetrainConstants.driveVelocityConversionFactor);

        // Configure rotation motor with conversion factors for radians
        configureMotor(rotationMotor, isDriveInverted,
            DrivetrainConstants.rotationPositionConversionFactor,
            DrivetrainConstants.rotationVelocityConversionFactor);

        driveMotor.clearFaults();
        rotationMotor.clearFaults();

        // CANcoder setup
        canCoder = new CANcoder(canCoderID);
        this.canCoderOffsetRadians = canCoderOffsetRadians;
        configureCanCoder();

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        // Sync rotation encoder to CANcoder absolute position on startup
        rotationEncoder.setPosition(getCANCoderRad());
        driveEncoder.setPosition(0);
    }

    private void configureMotor(
        SparkMax motorController,
        boolean isInverted,
        double positionConversionFactor,
        double velocityConversionFactor
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(isInverted)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureCanCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(config);
    }

    /** Returns the wheel angle in radians, corrected for the calibration offset. */
    private double getCANCoderRad() {
        double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
        double angle = (2.0 * Math.PI * absolutePosition) - canCoderOffsetRadians;
        return angle % (2.0 * Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMPS = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.rotationPositionRad = getCANCoderRad();
        inputs.rotationVelocityRadPerSec = rotationEncoder.getVelocity();
        inputs.rotationAppliedVolts = rotationAppliedVolts;
        inputs.rotationCurrentAmps = rotationMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = volts;
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setRotationVoltage(double volts) {
        rotationAppliedVolts = volts;
        rotationMotor.setVoltage(volts);
    }

    @Override
    public void stop() {
        driveAppliedVolts = 0.0;
        rotationAppliedVolts = 0.0;
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
    }
}
