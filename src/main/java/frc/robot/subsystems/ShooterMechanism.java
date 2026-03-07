package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Controls the shooter mechanism -- a dual-motor flywheel system with a kicker that
 * launches game pieces (FUEL) into the HUB.
 *
 * The shooter has THREE motors working together:
 *   1. Shooter Motor 1 (CAN ID 42) -- one side of the flywheel
 *   2. Shooter Motor 2 (CAN ID 43) -- other side of the flywheel
 *   3. Kicker Motor (CAN ID 44) -- pushes the game piece into the spinning flywheel
 *
 * How it works:
 *   - The two shooter motors spin up a flywheel to the desired speed.
 *   - Once the flywheel is at speed, the kicker motor pushes the game piece
 *     into the flywheel, which launches it toward the HUB.
 *   - The kicker always runs at 50% speed (hardcoded in setIOSpark).
 *
 * Both shooter motors receive the same speed value. If you want them to spin at
 * different speeds (e.g., to add spin to the game piece), you'd modify setIOSpark.
 *
 * All motors use brake mode and have current limits to prevent damage.
 *
 * WANT TO CHANGE shooter speed? See ShooterConstants.shooterSpeed in Constants.java.
 * WANT TO CHANGE the distance-to-speed lookup? See ShooterConstants.kDistanceToSpeedMap.
 * WANT TO CHANGE motor CAN IDs? See ShooterConstants in Constants.java.
 */
public class ShooterMechanism extends SubsystemBase {

    /** SparkMax controller for the first shooter flywheel motor. */
    private final SparkMax shooterSpark1;

    /** SparkMax controller for the second shooter flywheel motor. */
    private final SparkMax shooterSpark2;

    /** SparkMax controller for the kicker/feeder motor. */
    private final SparkMax kickerSpark;

    /**
     * Creates a new ShooterMechanism with the given motor ports.
     *
     * @param shooterSparkPort1 CAN ID of the first shooter motor
     * @param shooterSparkPort2 CAN ID of the second shooter motor
     * @param kickerSparkPort   CAN ID of the kicker motor
     */
    public ShooterMechanism(
        int shooterSparkPort1,
        int shooterSparkPort2,
        int kickerSparkPort
    ) {
        shooterSpark1 = new SparkMax(shooterSparkPort1, MotorType.kBrushless);
        shooterSpark2 = new SparkMax(shooterSparkPort2, MotorType.kBrushless);
        kickerSpark = new SparkMax(kickerSparkPort, MotorType.kBrushless);

        // Configure all three motors with brake mode and current limits
        configureMotor(shooterSpark1, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureMotor(shooterSpark2, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureMotor(kickerSpark, IdleMode.kBrake, ShooterConstants.currentLimit);
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
     * Sets the shooter flywheel speeds and activates the kicker.
     * Both shooter motors receive independent speed values (though they're usually the same).
     * The kicker always runs at 50% speed to feed the game piece into the flywheel.
     *
     * @param speed1 speed for shooter motor 1 (-1.0 to 1.0)
     * @param speed2 speed for shooter motor 2 (-1.0 to 1.0)
     */
    public void setIOSpark(double speed1, double speed2) {
        shooterSpark1.set(speed1);
        shooterSpark2.set(speed2);
        kickerSpark.set(.5); // Kicker always runs at 50% to feed the game piece
    }

    /**
     * Stops all three motors (both shooter motors and the kicker) immediately.
     * Called when shooter commands end.
     */
    public void stopIOSpark() {
        shooterSpark1.stopMotor();
        shooterSpark2.stopMotor();
        kickerSpark.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
