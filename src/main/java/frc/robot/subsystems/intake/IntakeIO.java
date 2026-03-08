package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what the intake hardware can do.
 *
 * This is part of the "IO pattern" -- we separate WHAT the mechanism does
 * (this interface) from HOW it talks to hardware (IntakeIOSparkMax for the
 * real robot, IntakeIOSim for simulation).
 *
 * The subsystem (IntakeMechanism) only talks to this interface, so it doesn't
 * care whether it's running on real hardware or in a simulator.
 *
 * WANT TO ADD A NEW SENSOR? Add a field to IntakeIOInputs and read it in
 * both IntakeIOSparkMax and IntakeIOSim.
 */
public interface IntakeIO {

    /**
     * Sensor readings from the intake hardware.
     *
     * The @AutoLog annotation tells AdvantageKit to automatically generate
     * logging code for these fields. Every 20ms, the values get recorded
     * so you can replay and analyze them later in AdvantageScope.
     */
    @AutoLog
    class IntakeIOInputs {
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
    default void updateInputs(IntakeIOInputs inputs) {}

    /**
     * Sets the intake motor speed.
     *
     * @param speed motor speed from -1.0 to 1.0 (positive = intake, negative = outtake)
     */
    default void setSpeed(double speed) {}

    /** Stops the intake motor immediately. */
    default void stop() {}
}
