package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Automatically rotates the robot to face an AprilTag while still allowing the driver
 * to manually control translation (forward/backward and strafing).
 *
 * How it works:
 *   - The Limelight tells us "tx" -- how many degrees left or right the AprilTag
 *     is from the center of the camera's view.
 *   - The code automatically spins the robot to bring tx to 0 (tag centered).
 *   - The driver can still drive forward/backward and sideways with the left stick
 *     while the auto-aim handles the spinning.
 *
 * The PID values (P=0.04) control how aggressively the robot spins toward the target.
 * The tolerance is 1 degree -- once the tag is within 1 degree of center, we're good.
 *
 * If no target is visible, the rotation speed is set to 0 (the robot only translates).
 *
 * Bound to: Driver controller Right Bumper (whileTrue -- aligns while held).
 *
 * WANT TO CHANGE alignment speed/accuracy? Adjust the turnPID values in this file.
 * WANT TO CHANGE the tolerance? Adjust turnPID.setTolerance().
 */
public class AlignToTag extends Command {
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY;
    private final SlewRateLimiter xLimiter, yLimiter;

    /** Controls how the robot spins to face the target. Tries to center the tag in the Limelight's view. */
    private final PIDController turnPID;

    /**
     * Creates a new AlignToTag command.
     *
     * @param drivetrain the swerve drivetrain subsystem
     * @param forwardX   supplier for forward/backward joystick input (manual control)
     * @param forwardY   supplier for left/right strafe joystick input (manual control)
     */
    public AlignToTag(
        SwerveSubsystem drivetrain,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY
    ) {
        this.drivetrain = drivetrain;
        this.forwardX = forwardX;
        this.forwardY = forwardY;

        // Smooth out the manual driving inputs so movement isn't jerky
        this.xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        this.yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);

        // Controls how the robot spins to face the tag.
        // 0.04 works well for our setup. Higher = spins faster, lower = gentler.
        this.turnPID = new PIDController(0.04, 0, 0);
        this.turnPID.setTolerance(1.0); // Consider "aligned" when within 1 degree

        addRequirements(drivetrain);
    }

    /**
     * Runs every 20ms: processes manual driving inputs for translation and uses
     * the Limelight's tx value for automatic rotation toward the AprilTag.
     */
    @Override
    public void execute() {
        // 1. Process manual joystick inputs for translation (forward/backward, left/right)
        double xSpeed = applyDeadbandAndLimiter(forwardX.getAsDouble(), OperatorConstants.xDeadband, xLimiter);
        double ySpeed = applyDeadbandAndLimiter(forwardY.getAsDouble(), OperatorConstants.yDeadband, yLimiter);

        // 2. Calculate rotation speed from vision
        double rotSpeed = 0;

        if (drivetrain.getLimelightHasTarget()) {
            // tx = horizontal angle offset to the target in degrees
            // Negative tx = target is to the left, positive = to the right
            double tx = drivetrain.getLimelightTx();

            // Negate because: if the target is to the right, we need to spin right
            // (which is negative in WPILib's coordinate system)
            rotSpeed = -turnPID.calculate(tx, 0);

            // Clamp to [-1, 1] range and scale to max angular velocity
            rotSpeed = Math.max(-1, Math.min(1, rotSpeed));
            rotSpeed *= DrivetrainConstants.maxAngularVelocity;
        }

        // 3. Drive the robot: manual translation + auto-rotation, always field-relative
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            true // Always field relative for ease of use during alignment
        );
    }

    /**
     * Ignores tiny stick movements (deadband) and smooths out acceleration.
     */
    private double applyDeadbandAndLimiter(double value, double deadband, SlewRateLimiter limiter) {
        value = Math.abs(value) < deadband ? 0 : value;
        return limiter.calculate(value) * DrivetrainConstants.maxVelocity;
    }
}
