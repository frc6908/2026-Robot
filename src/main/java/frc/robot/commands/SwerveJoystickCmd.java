package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The default teleop driving command -- this is what runs whenever the driver is
 * using the joysticks to control the robot.
 *
 * This command reads joystick inputs and converts them into swerve drive commands.
 * It applies several processing steps to make driving feel smooth and controllable:
 *
 *   1. DEADBANDS: Small joystick movements (within the deadband zone) are treated as
 *      zero. This prevents the robot from drifting when the driver's thumbs aren't
 *      perfectly centered on the sticks.
 *
 *   2. SLEW RATE LIMITING: Limits how quickly the speed can change. Instead of going
 *      from 0 to max speed instantly (which would jerk the robot), the speed ramps up
 *      smoothly. Think of it like the difference between slamming the gas pedal vs.
 *      gradually pressing it.
 *
 *   3. SPEED SLIDER: The left trigger acts as a "slow-down" control. Pulling it reduces
 *      ALL movement speeds proportionally. Fully pulled = 80% speed reduction.
 *      This gives the driver fine control for precise alignment.
 *
 * This command never finishes on its own (isFinished returns false) -- it runs
 * continuously as the drivetrain's default command until another command takes over.
 *
 * WANT TO CHANGE driving feel? Adjust deadbands in OperatorConstants, or max speeds
 * in DrivetrainConstants.
 * WANT TO CHANGE the slider range? Modify the sliderVal cap (currently 0.8 = 80% max reduction).
 */
public class SwerveJoystickCmd extends Command {
    public final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY, rotation, slider;

    /** Slew rate limiters -- limit how fast each axis can change (smooths acceleration). */
    private final SlewRateLimiter xLimiter, yLimiter, rLimiter;

    /**
     * Creates the default teleop drive command.
     *
     * @param swerveSubsystem the drivetrain subsystem to control
     * @param forwardX       supplier for forward/backward joystick axis (left stick Y)
     * @param forwardY       supplier for left/right strafe joystick axis (left stick X)
     * @param rotation       supplier for rotation joystick axis (right stick X)
     * @param slider         supplier for the speed slider (left trigger)
     */
    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY,
        DoubleSupplier rotation,
        DoubleSupplier slider
        ) {
        this.drivetrain = swerveSubsystem;
        this.forwardX = forwardX;
        this.forwardY = forwardY;
        this.rotation = rotation;
        this.slider = slider;

        // Create slew rate limiters. The rate is how many units per second the value
        // can change. maxAcceleration for translation, maxAngularAcceleration for rotation.
        xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        rLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration);

        // Tell the scheduler that this command uses the drivetrain.
        // This prevents other commands from using the drivetrain at the same time.
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){}

    /**
     * Called every 20ms. Reads joystick values, applies deadband + slew rate limiting,
     * applies the speed slider, and sends the processed speeds to the drivetrain.
     */
    @Override
    public void execute(){
        // Read raw joystick values (-1.0 to 1.0)
        double xSpeed = forwardX.getAsDouble();
        double ySpeed = forwardY.getAsDouble();
        double rot = rotation.getAsDouble();
        double sliderVal = slider.getAsDouble();

        // Apply deadband (ignore tiny movements) and slew rate limiting (smooth acceleration)
        // Then scale to the maximum speed in real units (m/s or rad/s)
        xSpeed = applyDeadbandAndLimiter(xSpeed, OperatorConstants.xDeadband, xLimiter, DrivetrainConstants.maxVelocity);
        ySpeed = applyDeadbandAndLimiter(ySpeed, OperatorConstants.yDeadband, yLimiter, DrivetrainConstants.maxVelocity);
        rot = applyDeadbandAndLimiter(rot, OperatorConstants.rDeadband, rLimiter, DrivetrainConstants.maxAngularVelocity);

        // Speed slider: left trigger (0.0 = not pulled, 1.0 = fully pulled).
        // Cap at 0.8 so the robot never slows to a complete stop from the slider alone.
        // The formula (1 - sliderVal) means: fully released = full speed, pulled = reduced speed.
        sliderVal = Math.min(sliderVal, 0.8);
        xSpeed *= (1 - sliderVal);
        ySpeed *= (1 - sliderVal);
        rot *= (1 - sliderVal);

        // Send the processed speeds to the drivetrain
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rot,
            SwerveSubsystem.fieldRelativeStatus
        );
    }

    /**
     * Applies a deadband and slew rate limiter to a joystick value, then scales it.
     *
     * @param value    raw joystick value (-1.0 to 1.0)
     * @param deadband values below this threshold are treated as 0
     * @param limiter  limits how fast the value can change per second
     * @param maxSpeed multiplier to convert the 0-1 range to real units (m/s or rad/s)
     * @return the processed speed value in real units
     */
    public double applyDeadbandAndLimiter(
        double value,
        double deadband,
        SlewRateLimiter limiter,
        double maxSpeed
    ) {
        value = Math.abs(value) < deadband ? 0 : value;
        return limiter.calculate(value)*maxSpeed;
    }

    @Override
    public void end(boolean interrupted){}

    /** Never finishes -- runs continuously as the default command. */
    @Override
    public boolean isFinished(){
        return false;
    }
}
