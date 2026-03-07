package frc.robot.subsystems;

import frc.robot.Constants.BarConstants;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class BarMechanism extends SubsystemBase {
    private final SparkMax barSpark;
    private final SparkMax barSpark2;

    public BarMechanism(
        int barSparkPort,
        int barSpark2Port
        
    ) {
        barSpark = new SparkMax(barSparkPort, MotorType.kBrushless); 
        barSpark2 = new SparkMax(barSpark2Port, MotorType.kBrushless); 

        configureMotor(barSpark, IdleMode.kBrake, BarConstants.currentLimit);
        configureMotor(barSpark2, IdleMode.kBrake, BarConstants.currentLimit);

        
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

    public void setBarSpark(double speed) {
        barSpark.set(speed);
    }

    public void setBarSpark2(double speed) {
        barSpark2.set(speed);
    }

    public void stopBarSpark() {
        barSpark.stopMotor();
    }

    public void stopBarSpark2() {
        barSpark2.stopMotor();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
