package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface that defines what the shooter hardware can do.
 *
 * The shooter has THREE motors:
 *   - Two flywheel motors (shooter 1 and shooter 2) that spin up to launch game pieces
 *   - One kicker motor that feeds the game piece into the spinning flywheel
 *
 * This interface separates WHAT the mechanism does from HOW it talks to hardware,
 * allowing us to swap between real hardware and simulation.
 *
 * WANT TO ADD A NEW SENSOR? Add a field to ShooterIOInputs and read it in
 * both ShooterIOSparkMax and ShooterIOSim.
 */
public interface ShooterIO {

    /**
     * Sensor readings from all three shooter motors.
     *
     * The @AutoLog annotation tells AdvantageKit to automatically generate
     * logging code for these fields.
     */
    @AutoLog
    class ShooterIOInputs {
        /** Speed applied to shooter motor 1 (-1.0 to 1.0). */
        public double shooter1AppliedSpeed = 0.0;

        /** Velocity of shooter motor 1 (radians per second). */
        public double shooter1VelocityRadPerSec = 0.0;

        /** Current draw of shooter motor 1 (Amps). */
        public double shooter1CurrentAmps = 0.0;

        /** Speed applied to shooter motor 2 (-1.0 to 1.0). */
        public double shooter2AppliedSpeed = 0.0;

        /** Velocity of shooter motor 2 (radians per second). */
        public double shooter2VelocityRadPerSec = 0.0;

        /** Current draw of shooter motor 2 (Amps). */
        public double shooter2CurrentAmps = 0.0;

        /** Speed applied to the kicker motor (-1.0 to 1.0). */
        public double kickerAppliedSpeed = 0.0;

        /** Velocity of the kicker motor (radians per second). */
        public double kickerVelocityRadPerSec = 0.0;

        /** Current draw of the kicker motor (Amps). */
        public double kickerCurrentAmps = 0.0;
    }

    /**
     * Reads the latest sensor values from all three motors.
     * Called every 20ms by the subsystem's periodic() method.
     */
    default void updateInputs(ShooterIOInputs inputs) {}

    /**
     * Sets the speed of both shooter flywheel motors.
     *
     * @param speed1 speed for motor 1 (-1.0 to 1.0)
     * @param speed2 speed for motor 2 (-1.0 to 1.0)
     */
    default void setShooterSpeed(double speed1, double speed2) {}

    /**
     * Sets the speed of the kicker/feeder motor.
     *
     * @param speed kicker speed (-1.0 to 1.0)
     */
    default void setKickerSpeed(double speed) {}

    /** Stops all three motors (both shooter motors and the kicker). */
    default void stop() {}
}
