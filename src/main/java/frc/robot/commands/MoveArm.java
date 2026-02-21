package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeMechanism;

/**
 * Moves the algae arm up or down at a fixed speed.
 *
 * The direction is controlled by the "movingUp" parameter passed into the
 * constructor. This command runs as long as the button is held (whileTrue),
 * and stops the arm motor when the button is released.
 *
 * There's commented-out code for "soft stops" -- a feature that would slow
 * the arm down as it approaches its upper or lower limits to prevent slamming.
 * This can be re-enabled once the arm encoder is working correctly.
 *
 * WANT TO CHANGE the arm speed? → Edit AlgaeConstants.algaeArmSpeed in Constants.java
 * WANT TO ENABLE soft stops so the arm slows near its limits?
 *   1. First fix the encoder in AlgaeMechanism.getArmEncoderValue() (uncomment real reading)
 *   2. Uncomment the "distanceToEdge" and "speed" lines in execute() below
 *   3. Adjust the limit values in AlgaeConstants (upperLimitRotation, lowerLimitRotation)
 * WANT THE ARM TO GO TO A SPECIFIC POSITION automatically?
 *   You'd need to add a PID controller (like the swerve modules use) to smoothly
 *   move the arm to a target encoder value instead of just setting a fixed speed.
 */
public class MoveArm extends Command {
    private final AlgaeMechanism m_algaeMech;
    // true = arm goes up, false = arm goes down
    private final boolean movingUp;

    /**
     * Creates a new MoveArm command.
     *
     * @param algaeMech  the algae mechanism subsystem
     * @param movingUp   true to move the arm up, false to move it down
     */
    public MoveArm(AlgaeMechanism algaeMech, boolean movingUp) {
        m_algaeMech = algaeMech;
        this.movingUp = movingUp;
        // Declare that this command needs the algae mechanism.
        // This prevents the intake and arm from getting conflicting commands.
        addRequirements(m_algaeMech);
    }

    @Override
    public void initialize() {}

    /**
     * Runs every 20ms. Sets the arm motor speed based on direction.
     * Moving up uses negative speed (motor direction is reversed for "up").
     */
    @Override
    public void execute() {
        double speed = AlgaeConstants.algaeArmSpeed;
        if (movingUp) {
            // Soft stop logic (commented out until encoder works):
            // distanceToEdge = AlgaeConstants.upperLimitRotation - m_algaeMech.getArmEncoderValue();
            // speed = Math.min(1, distanceToEdge/AlgaeConstants.softStopDistance) * -AlgaeConstants.algaeArmSpeed;

            // Negative speed = arm moves up (motor is mounted so negative = up)
            speed *= -1;
        }
        else {
            // Soft stop logic for downward motion (commented out):
            // distanceToEdge = m_algaeMech.getArmEncoderValue() - AlgaeConstants.lowerLimitRotation;
            // speed = Math.min(1, distanceToEdge/AlgaeConstants.softStopDistance) * AlgaeConstants.algaeArmSpeed;
        }
        m_algaeMech.setAlgaeArmSpark(speed);
    }

    /**
     * When the button is released (or the command is interrupted by another command),
     * stop the arm motor so it doesn't keep moving.
     */
    @Override
    public void end(boolean interrupted) {
        m_algaeMech.stopAlgaeArmSpark();
    }

    /**
     * Never finishes on its own -- runs until the button is released.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
