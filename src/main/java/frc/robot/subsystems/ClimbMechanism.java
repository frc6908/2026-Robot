package frc.robot.subsystems;

import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Controls the climbing motor -- the mechanism that lifts the robot onto the chain
 * at the end of a match for bonus points.
 *
 * This is a simple single-motor subsystem. Positive speed climbs up (retracts the
 * hook/arm), negative speed goes down (extends the hook/arm). The motor is set to
 * brake mode so the robot stays in place once it has climbed.
 *
 * The climb mechanism is typically used only in the last 20 seconds of a match
 * during the "endgame" period.
 *
 * WANT TO CHANGE climb speed? The speed is set in the Climb and ClimbDown commands
 * (currently 75% in both directions).
 * WANT TO CHANGE the motor's CAN ID? See ClimbConstants.climbSparkPort1 in Constants.java.
 */
public class ClimbMechanism extends SubsystemBase {

    /** The SparkMax motor controller for the climb motor. */
    private final SparkMax climbSpark1;

    /**
     * Creates a new ClimbMechanism with the given motor port.
     *
     * @param climbSparkPort1 CAN ID of the climb SparkMax motor controller
     */
    public ClimbMechanism(
        int climbSparkPort1
    ) {
        climbSpark1 = new SparkMax(climbSparkPort1, MotorType.kBrushless);
        configureMotor(climbSpark1, IdleMode.kBrake, ClimbConstants.currentLimit);
    }

    /**
     * Configures a SparkMax motor controller with brake mode and a current limit.
     *
     * @param spark        the SparkMax to configure
     * @param idleMode     kBrake or kCoast
     * @param currentLimit maximum current draw in Amps
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
     * Sets the climb motor speed. Positive = climb up, negative = go down.
     *
     * @param speed1 motor speed from -1.0 to 1.0
     */
    public void setIOSpark(double speed1) {
        climbSpark1.set(speed1);
    }

    /**
     * Stops the climb motor immediately. Called when climb commands end.
     */
    public void stopIOSpark() {
        climbSpark1.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
