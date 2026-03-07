package frc.robot.subsystems.swerve;

/**
 * Simulation implementation of VisionIO -- returns no-target defaults.
 *
 * In simulation, the Limelight camera is not available, so this implementation
 * simply reports "no target found." The robot will still drive and run auto
 * routines, but auto-aim and vision-based pose correction won't work.
 *
 * This can be enhanced later to calculate fake targets from the simulated
 * robot pose if needed.
 *
 * Only used in simulation. On the real robot, VisionIOLimelight is used instead.
 */
public class VisionIOSim implements VisionIO {

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // No target in simulation by default
        inputs.hasTarget = false;
        inputs.tx = 0.0;
        inputs.targetId = -1;
        inputs.targetDistanceMeters = -1.0;
        inputs.botpose = new double[7];
    }
}
