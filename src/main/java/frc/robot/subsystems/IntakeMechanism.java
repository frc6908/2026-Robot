package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Controls the intake roller motor -- the mechanism that sucks game pieces into the robot
 * or spits them back out.
 *
 * This is one of the simplest subsystems: a single SparkMax motor controller running
 * a brushless (NEO) motor. Positive speed sucks game pieces in (intake), negative speed
 * spits them out (outtake).
 *
 * The motor is set to brake mode so the rollers stop immediately when the command ends,
 * preventing game pieces from rolling back out. A current limit protects the motor from
 * burning out if a game piece gets jammed.
 *
 * WANT TO CHANGE intake/outtake speed? See IntakeConstants in Constants.java.
 * WANT TO CHANGE the motor's CAN ID? See IntakeConstants.ioSparkPort in Constants.java.
 */
public class IntakeMechanism extends SubsystemBase {

    /** The SparkMax motor controller for the intake roller. */
    private final SparkMax ioSpark;

    /**
     * Creates a new IntakeMechanism with the given motor port.
     *
     * @param ioSparkPort CAN ID of the intake SparkMax motor controller
     */
    public IntakeMechanism(
        int ioSparkPort
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        configureMotor(ioSpark, IdleMode.kBrake, IntakeConstants.currentLimit);
    }

    /**
     * Configures a SparkMax motor controller with brake mode and a current limit.
     *
     * @param spark        the SparkMax to configure
     * @param idleMode     kBrake (motor resists movement when not powered) or kCoast
     * @param currentLimit maximum current draw in Amps (protects the motor)
     */
    public void configureMotor(
        SparkMax spark,
        IdleMode idleMode,
        int currentLimit
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(idleMode)
            .smartCurrentLimit(currentLimit);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the intake motor speed. Positive = intake (suck in), negative = outtake (spit out).
     *
     * @param speed motor speed from -1.0 to 1.0
     */
    public void setIOSpark(double speed) {
        ioSpark.set(speed);
    }

    /**
     * Stops the intake motor immediately. Called when intake/outtake commands end.
     */
    public void stopIOSpark() {
        ioSpark.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
