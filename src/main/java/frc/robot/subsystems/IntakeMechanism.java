package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeMechanism extends SubsystemBase {
    private final SparkMax ioSpark;

    public IntakeMechanism(
        int ioSparkPort
        
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        

        configureMotor(ioSpark, IdleMode.kBrake, IntakeConstants.currentLimit);

        
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

    public void setIOSpark(double speed) {
        ioSpark.set(speed);
    }

    public void stopIOSpark() {
        ioSpark.stopMotor();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
