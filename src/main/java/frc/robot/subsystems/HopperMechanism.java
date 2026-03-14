package frc.robot.subsystems;

import frc.robot.Constants.HopperConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperMechanism extends SubsystemBase {
    private final SparkMax hopperSpark;

    public HopperMechanism(
        int hopperSparkPort

    ) {
        hopperSpark = new SparkMax(hopperSparkPort, MotorType.kBrushless);

        configureMotor(hopperSpark, IdleMode.kBrake, HopperConstants.currentLimit);


    }

    public void configureMotor(
        SparkMax spark,
        IdleMode idleMode,
        int currentLimit
        ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(idleMode)
            .smartCurrentLimit(currentLimit);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setHopperSpark(double speed) {
        hopperSpark.set(speed);
    }

    public void stopHopperSpark() {
        hopperSpark.stopMotor();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
}
