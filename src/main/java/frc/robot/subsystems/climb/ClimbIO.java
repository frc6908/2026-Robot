package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what the climb hardware can do.
 *
 * This is part of the "IO pattern" -- we separate WHAT the mechanism does
 * (this interface) from HOW it talks to hardware (ClimbIOSparkMax for the
 * real robot, ClimbIOSim for simulation).
 *
 * WANT TO ADD A NEW SENSOR? Add a field to ClimbIOInputs and read it in
 * both ClimbIOSparkMax and ClimbIOSim.
 */
public interface ClimbIO {

    /**
     * Sensor readings from the climb hardware.
     *
     * The @AutoLog annotation tells AdvantageKit to automatically generate
     * logging code for these fields.
     */
    @AutoLog
    class ClimbIOInputs {
        /** The speed being applied to the motor (-1.0 to 1.0). */
        public double appliedSpeed = 0.0;

        /** How fast the motor is spinning (in radians per second). */
        public double velocityRadPerSec = 0.0;

        /** How much current the motor is drawing (in Amps). */
        public double currentAmps = 0.0;
    }

    /**
     * Reads the latest sensor values from the hardware into the inputs object.
     * Called every 20ms by the subsystem's periodic() method.
     */
    default void updateInputs(ClimbIOInputs inputs) {}

    /**
     * Sets the climb motor speed.
     *
     * @param speed motor speed from -1.0 to 1.0 (positive = climb up, negative = go down)
     */
    default void setSpeed(double speed) {}

    /** Stops the climb motor immediately. */
    default void stop() {}
}
