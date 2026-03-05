package frc.robot.subsystems;

import frc.robot.Constants.ClimbConstants;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbMechanism extends SubsystemBase {
    private final SparkMax climbSpark1;


    public ClimbMechanism(
        int climbSparkPort1
        
    ) {
        climbSpark1 = new SparkMax(climbSparkPort1, MotorType.kBrushless);
        configureMotor(climbSpark1, IdleMode.kBrake, ClimbConstants.currentLimit);


        
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

    public void setIOSpark(double speed1) {
        climbSpark1.set(speed1);
    }

    public void stopIOSpark() {
        climbSpark1.stopMotor();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
