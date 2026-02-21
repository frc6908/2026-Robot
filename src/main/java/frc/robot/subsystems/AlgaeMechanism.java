package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The AlgaeMechanism subsystem controls the arm and intake/outtake rollers
 * used to pick up and score algae game pieces.
 *
 * It has two motors:
 *   1. IO (Intake/Outtake) motor: spins rollers to suck in or spit out algae
 *   2. Arm motor: pivots the arm up and down to reach different heights
 *
 * It also has a quadrature encoder on the arm to track its position.
 *
 * WANT TO ADD a new motor or sensor to this mechanism?
 *   1. Add the CAN ID or port number to Constants.AlgaeConstants
 *   2. Create a new field here (like "private final SparkMax newMotor;")
 *   3. Initialize it in the constructor
 *   4. Add set/stop methods for it (follow the pattern of setIOSpark/stopIOSpark)
 *   5. Create a new Command class that calls your new methods
 *   6. Bind it to a button in RobotContainer.configureBindings()
 *
 * WANT TO FIX the arm encoder?
 *   The getArmEncoderValue() method currently returns a hardcoded number.
 *   Uncomment the real encoder reading and delete the hardcoded line.
 */
public class AlgaeMechanism extends SubsystemBase {
    // The roller motor -- spins one way to intake, the other way to outtake
    private final SparkMax ioSpark;
    // The arm motor -- pivots the whole mechanism up and down
    private final SparkMax algaeArmSpark;
    // Encoder to track how far the arm has rotated (so we know its angle)
    private final Encoder algaeArmEncoder;

    /**
     * Creates the algae mechanism with the given motor CAN IDs.
     *
     * @param ioSparkPort        CAN ID for the intake/outtake motor
     * @param algaeArmSparkPort  CAN ID for the arm pivot motor
     */
    public AlgaeMechanism(
        int ioSparkPort,
        int algaeArmSparkPort
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        algaeArmSpark = new SparkMax(algaeArmSparkPort, MotorType.kBrushless);
        // Quadrature encoder uses two channels (A and B) to detect both
        // position and direction of rotation
        algaeArmEncoder = new Encoder(AlgaeConstants.algaeArmEncoderChannelA, AlgaeConstants.algaeArmEncoderChannelB);

        // Configure both motors:
        //   - Brake mode: motors resist movement when not powered (holds position)
        //   - Current limit: prevents the motor from drawing too much power,
        //     which protects the motor and gears from damage
        configureMotor(ioSpark, IdleMode.kBrake, AlgaeConstants.currentLimit);
        configureMotor(algaeArmSpark, IdleMode.kBrake, AlgaeConstants.currentLimit);
    }

    /**
     * Applies common settings to a SparkMax motor controller.
     *
     * @param spark         the motor controller to configure
     * @param idleMode      brake (resists motion) or coast (spins freely) when idle
     * @param currentLimit  max amps the motor is allowed to draw
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
     * Sets the intake/outtake roller speed.
     * Positive = intake (sucking in), negative = outtake (spitting out).
     *
     * @param speed  motor power from -1.0 to 1.0
     */
    public void setIOSpark(double speed) {
        ioSpark.set(speed);
    }

    /** Stops the intake/outtake roller. */
    public void stopIOSpark() {
        ioSpark.stopMotor();
    }

    /**
     * Sets the arm motor speed. Positive/negative controls direction.
     *
     * @param speed  motor power from -1.0 to 1.0
     */
    public void setAlgaeArmSpark(double speed) {
        algaeArmSpark.set(speed);
    }

    /** Stops the arm motor. */
    public void stopAlgaeArmSpark() {
        algaeArmSpark.stopMotor();
    }

    /**
     * Returns the current arm position from the encoder.
     * NOTE: Currently returns a hardcoded value (3.2425) -- the real encoder
     * reading is commented out. This should be fixed when the encoder is working.
     */
    public double getArmEncoderValue() {
        // return algaeArmEncoder.getDistance();
        return 3.2425;
    }

    /** Resets the arm encoder to 0 (current position becomes the new "zero"). */
    public void resetArmEncoder() {
        algaeArmEncoder.reset();
    }

    /**
     * Called every 20ms. Publishes the arm encoder value to SmartDashboard
     * so we can see the arm position from the laptop.
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder Val", getArmEncoderValue());
    }
}
