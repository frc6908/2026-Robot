package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTag extends Command {
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY;
    private final SlewRateLimiter xLimiter, yLimiter;
    private final PIDController turnPID;

    private static final Set<Integer> redHubTags  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final Set<Integer> blueHubTags = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

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
        this.turnPID = new PIDController(0.02, 0, 0); // halved from 0.04 to account for 2π maxAngularVelocity
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

        if (drivetrain.getLimelightHasTarget()) {
            var alliance = DriverStation.getAlliance();
            Set<Integer> targetTags = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
                ? redHubTags : blueHubTags;

            int tid = drivetrain.getLimelightTid();
            if (targetTags.contains(tid)) {
                // tx is the horizontal angle offset to the target in degrees (negative = left, positive = right)
                double tx = drivetrain.getLimelightTx();

                if (Math.abs(tx) > 1.0) {
                    // Negate: tx > 0 means target is right, WPILib CCW is positive, so we turn CW (negative)
                    rotSpeed = -turnPID.calculate(tx, 0);

                    // Clamp and scale to max angular velocity
                    rotSpeed = Math.max(-1, Math.min(1, rotSpeed));
                    rotSpeed *= DrivetrainConstants.maxAngularVelocity;
                }
            }
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