package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;

/**
 * Hardware implementation of GyroIO -- talks to the real NavX gyroscope.
 *
 * The NavX is connected via the MXP SPI port on the RoboRIO. It provides
 * yaw (heading) and yaw rate measurements. A 1-second delay on startup
 * allows the NavX to finish its internal calibration.
 *
 * Only used on the real robot. In simulation, GyroIOSim is used instead.
 */
public class GyroIONavX implements GyroIO {

    /** The NavX gyroscope connected via MXP SPI. */
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        // Delay NavX reset by 1 second to allow it to finish booting up.
        // The NavX needs time to calibrate its internal sensors after power-on.
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                navX.reset();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navX.isConnected();
        inputs.yawDegrees = -navX.getYaw(); // Negate to match WPILib convention
        inputs.yawRateDegreesPerSec = -navX.getRate(); // Negate to match WPILib convention
    }

    @Override
    public void reset() {
        navX.reset();
    }
}
