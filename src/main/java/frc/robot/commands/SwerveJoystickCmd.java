package frc.robot.commands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The main teleop driving command. This runs the entire time the driver is
 * controlling the robot and translates joystick inputs into swerve drive movements.
 *
 * This command is set as the "default command" for the SwerveSubsystem, meaning
 * it runs automatically whenever no other command (like an auto path) is using
 * the drivetrain.
 *
 * Features:
 *   - Deadband: ignores tiny joystick movements so the robot doesn't drift
 *   - Slew rate limiting: smooths out jerky joystick inputs so the robot
 *     accelerates gradually instead of snapping to full speed instantly
 *   - Speed slider: the left trigger reduces max speed for finer control
 *
 * WANT TO CHANGE how the robot drives during teleop?
 *   - Max speed → change DrivetrainConstants.maxVelocity in Constants.java
 *   - How quickly it accelerates → change DrivetrainConstants.maxAcceleration in Constants.java
 *   - Joystick dead zone → change deadband values in Constants.OperatorConstants
 *   - How much the trigger slows you down → change the 0.8 cap in execute() below
 */
public class SwerveJoystickCmd extends Command {
    public final SwerveSubsystem drivetrain;

    // These are "suppliers" -- instead of storing a fixed number, they store a
    // function that gives us a fresh number every time we call it. This way we
    // get the current joystick position each time execute() runs (every 20ms).
    private final DoubleSupplier forwardX, forwardY, rotation, slider;

    // Slew rate limiters prevent the speed from changing too quickly.
    // Think of it like a speed limit on acceleration -- the robot can't go from
    // 0 to full speed instantly, it has to ramp up smoothly.
    private final SlewRateLimiter xLimiter, yLimiter, rLimiter;

    /**
     * Creates a new SwerveJoystickCmd.
     *
     * @param swerveSubsystem  the drivetrain subsystem to control
     * @param forwardX         supplier for forward/backward joystick input (left stick Y)
     * @param forwardY         supplier for left/right strafe joystick input (left stick X)
     * @param rotation         supplier for rotation joystick input (right stick X)
     * @param slider           supplier for the speed slider (left trigger axis)
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

        // Create slew rate limiters with the max acceleration values from Constants.
        // The limiter won't let the output change faster than this rate per second.
        xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        rLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration);

        // Tell the CommandScheduler that this command uses the drivetrain.
        // This prevents two commands from trying to control the drivetrain at the same time.
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){}

    /**
     * Runs every 20ms while this command is active. Reads the joystick values,
     * applies deadband and smoothing, then sends the drive command.
     */
    @Override
    public void execute(){
        // Read the current joystick positions
        double xSpeed = forwardX.getAsDouble();
        double ySpeed = forwardY.getAsDouble();
        double rot = rotation.getAsDouble();
        double sliderVal = slider.getAsDouble();

        // Apply deadband (ignore tiny values) and slew rate limiting (smooth acceleration),
        // then scale to the max speed. The result is a speed in m/s or rad/s.
        xSpeed = applyDeadbandAndLimiter(xSpeed, OperatorConstants.xDeadband, xLimiter, DrivetrainConstants.maxVelocity);
        ySpeed = applyDeadbandAndLimiter(ySpeed, OperatorConstants.yDeadband, yLimiter, DrivetrainConstants.maxVelocity);
        rot = applyDeadbandAndLimiter(rot, OperatorConstants.rDeadband, rLimiter, DrivetrainConstants.maxAngularVelocity);

        // The left trigger acts as a speed reducer (slider).
        // Pulling the trigger more reduces the max speed, giving finer control.
        // Cap at 0.8 so the robot never goes all the way to 0 speed.
        //
        // WANT TO CHANGE the slider behavior?
        //   - Change 0.8 to a lower number to allow more speed reduction (e.g., 0.9 = can slow to 10%)
        //   - Change 0.8 to a higher number to limit how much it can slow down (e.g., 0.5 = can only slow to 50%)
        sliderVal = Math.min(sliderVal, 0.8);
        xSpeed *= (1 - sliderVal);
        ySpeed *= (1 - sliderVal);
        rot *= (1 - sliderVal);

        // Send the final speeds to the drivetrain
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rot,
            SwerveSubsystem.fieldRelativeStatus
        );
    }

    /**
     * Helper method that applies deadband and slew rate limiting to a joystick value.
     *
     * @param value     the raw joystick value (-1.0 to 1.0)
     * @param deadband  values below this threshold are treated as 0
     * @param limiter   slew rate limiter to smooth the output
     * @param maxSpeed  maximum speed to scale the output to (m/s or rad/s)
     * @return          the processed speed value ready to send to the drivetrain
     */
    public double applyDeadbandAndLimiter(
        double value,
        double deadband,
        SlewRateLimiter limiter,
        double maxSpeed
    ) {
        // If the joystick value is smaller than the deadband, treat it as zero
        value = Math.abs(value) < deadband ? 0 : value;
        // Run through the slew rate limiter (prevents sudden changes), then
        // multiply by max speed to convert from joystick units (-1 to 1) to
        // real-world units (m/s or rad/s)
        return limiter.calculate(value)*maxSpeed;
    }

    @Override
    public void end(boolean interrupted){}

    /**
     * Returns false because this command should never end on its own -- it runs
     * continuously as the default command until something else takes over the drivetrain.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
}