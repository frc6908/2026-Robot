package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the shooter mechanism -- a dual-motor flywheel system with a kicker that
 * launches game pieces (FUEL) into the HUB.
 *
 * This subsystem uses the IO pattern: instead of talking to hardware directly,
 * it talks to a ShooterIO interface. On the real robot, that interface is backed
 * by ShooterIOSparkMax (real motors). In simulation, it's backed by ShooterIOSim
 * (physics simulation). The subsystem code is identical either way.
 *
 * The shooter has THREE motors working together:
 *   1. Shooter Motor 1 (CAN ID 42) -- one side of the flywheel
 *   2. Shooter Motor 2 (CAN ID 43) -- other side of the flywheel
 *   3. Kicker Motor (CAN ID 44) -- pushes the game piece into the spinning flywheel
 *
 * WANT TO CHANGE shooter speed? See ShooterConstants.shooterSpeed in Constants.java.
 * WANT TO CHANGE the distance-to-speed lookup? See ShooterConstants.kDistanceToSpeedMap.
 * WANT TO CHANGE motor CAN IDs? See ShooterConstants in Constants.java.
 */
public class ShooterMechanism extends SubsystemBase {

    /** The IO layer -- could be real hardware or simulation. */
    private final ShooterIO io;

    /** Logged sensor readings, updated every 20ms. */
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /**
     * Creates a new ShooterMechanism with the given IO implementation.
     *
     * @param io the hardware interface (ShooterIOSparkMax for real, ShooterIOSim for sim)
     */
    public ShooterMechanism(ShooterIO io) {
        this.io = io;
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
        io.setShooterSpeed(speed1, speed2);
        io.setKickerSpeed(0.5); // Kicker always runs at 50% to feed the game piece
    }

    /**
     * Stops all three motors (both shooter motors and the kicker) immediately.
     * Called when shooter commands end.
     */
    public void stopIOSpark() {
        io.stop();
    }

    @Override
    public void periodic() {
        // Read the latest sensor values from hardware (or sim)
        io.updateInputs(inputs);
        // Log all sensor values to AdvantageKit for replay and analysis
        Logger.processInputs("Shooter", inputs);
    }
}
