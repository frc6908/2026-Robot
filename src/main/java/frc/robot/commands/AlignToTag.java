package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;

public class AlignToTag extends Command {
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY;
    private final SlewRateLimiter xLimiter, yLimiter;
    private final PIDController turnPID;

    public AlignToTag(
        SwerveSubsystem drivetrain,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY
    ) {
        this.drivetrain = drivetrain;
        this.forwardX = forwardX;
        this.forwardY = forwardY;

        this.xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        this.yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);

        // PID Controller for turning:
        // P = 0.1 is a good starting point for "Degrees" error
        this.turnPID = new PIDController(0.04, 0, 0); 
        this.turnPID.setTolerance(1.0); // 1 degree tolerance

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // 1. Get Joystick Inputs (Driving)
        double xSpeed = applyDeadbandAndLimiter(forwardX.getAsDouble(), OperatorConstants.xDeadband, xLimiter);
        double ySpeed = applyDeadbandAndLimiter(forwardY.getAsDouble(), OperatorConstants.yDeadband, yLimiter);
        
        // 2. Calculate Rotation (Vision)
        double rotSpeed = 0;
        PhotonPipelineResult result = drivetrain.getCameraResult();

        if (result.hasTargets()) {
            // Get the yaw (offset angle) to the best target
            double currentYaw = result.getBestTarget().getYaw();
            
            // Calculate PID output to turn towards 0 yaw (centered)
            // We aim for 0, current is yaw.
            rotSpeed = turnPID.calculate(currentYaw, 0);

            // Invert if necessary depending on your motor config, usually PID calculates correctly for error
            // Hower, Swerve expects Radians/Sec. 
            // The PID output here is essentially a "speed percentage" or arbitrary unit, 
            // so we scale it to max angular velocity
            
             // Clamp speed to prevent violent oscillations
             rotSpeed = Math.max(-1, Math.min(1, rotSpeed));
             rotSpeed *= DrivetrainConstants.maxAngularVelocity; 

        } else {
            // If no target, stop rotating (or you could pass manual rotation here)
            rotSpeed = 0;
        }

        // 3. Drive
        drivetrain.drive(
            xSpeed,
            ySpeed,
            rotSpeed,
            true // Always field relative for ease of use
        );
    }

    private double applyDeadbandAndLimiter(double value, double deadband, SlewRateLimiter limiter) {
        value = Math.abs(value) < deadband ? 0 : value;
        return limiter.calculate(value) * DrivetrainConstants.maxVelocity;
    }
}