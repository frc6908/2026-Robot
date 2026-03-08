package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the intake roller motor -- the mechanism that sucks game pieces into the robot
 * or spits them back out.
 *
 * This subsystem uses the IO pattern: instead of talking to hardware directly,
 * it talks to an IntakeIO interface. On the real robot, that interface is backed
 * by IntakeIOSparkMax (real motors). In simulation, it's backed by IntakeIOSim
 * (physics simulation). The subsystem code is identical either way.
 *
 * Positive speed sucks game pieces in (intake), negative speed
 * spits them out (outtake).
 *
 * WANT TO CHANGE intake/outtake speed? See IntakeConstants in Constants.java.
 * WANT TO CHANGE the motor's CAN ID? See IntakeConstants.ioSparkPort in Constants.java.
 */
public class IntakeMechanism extends SubsystemBase {

    /** The IO layer -- could be real hardware or simulation. */
    private final IntakeIO io;

    /** Logged sensor readings, updated every 20ms. */
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    /**
     * Creates a new IntakeMechanism with the given IO implementation.
     *
     * @param io the hardware interface (IntakeIOSparkMax for real, IntakeIOSim for sim)
     */
    public IntakeMechanism(IntakeIO io) {
        this.io = io;
    }

    /**
     * Sets the intake motor speed. Positive = intake (suck in), negative = outtake (spit out).
     *
     * @param speed motor speed from -1.0 to 1.0
     */
    public void setIOSpark(double speed) {
        io.setSpeed(speed);
    }

    /**
     * Stops the intake motor immediately. Called when intake/outtake commands end.
     */
    public void stopIOSpark() {
        io.stop();
    }

    @Override
    public void periodic() {
        // Read the latest sensor values from hardware (or sim)
        io.updateInputs(inputs);
        // Log all sensor values to AdvantageKit for replay and analysis
        Logger.processInputs("Intake", inputs);
    }
}
