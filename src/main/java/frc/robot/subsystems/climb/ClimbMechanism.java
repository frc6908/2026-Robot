package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the climbing motor -- the mechanism that lifts the robot onto the chain
 * at the end of a match for bonus points.
 *
 * This subsystem uses the IO pattern: instead of talking to hardware directly,
 * it talks to a ClimbIO interface. On the real robot, that interface is backed
 * by ClimbIOSparkMax (real motors). In simulation, it's backed by ClimbIOSim
 * (physics simulation). The subsystem code is identical either way.
 *
 * Positive speed climbs up (retracts the hook/arm), negative speed goes down
 * (extends the hook/arm). The motor is set to brake mode so the robot stays
 * in place once it has climbed.
 *
 * WANT TO CHANGE climb speed? The speed is set in the Climb and ClimbDown commands
 * (currently 75% in both directions).
 * WANT TO CHANGE the motor's CAN ID? See ClimbConstants.climbSparkPort1 in Constants.java.
 */
public class ClimbMechanism extends SubsystemBase {

    /** The IO layer -- could be real hardware or simulation. */
    private final ClimbIO io;

    /** Logged sensor readings, updated every 20ms. */
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    /**
     * Creates a new ClimbMechanism with the given IO implementation.
     *
     * @param io the hardware interface (ClimbIOSparkMax for real, ClimbIOSim for sim)
     */
    public ClimbMechanism(ClimbIO io) {
        this.io = io;
    }

    /**
     * Sets the climb motor speed. Positive = climb up, negative = go down.
     *
     * @param speed1 motor speed from -1.0 to 1.0
     */
    public void setIOSpark(double speed1) {
        io.setSpeed(speed1);
    }

    /**
     * Stops the climb motor immediately. Called when climb commands end.
     */
    public void stopIOSpark() {
        io.stop();
    }

    @Override
    public void periodic() {
        // Read the latest sensor values from hardware (or sim)
        io.updateInputs(inputs);
        // Log all sensor values to AdvantageKit for replay and analysis
        Logger.processInputs("Climb", inputs);
    }
}
