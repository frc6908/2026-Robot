package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterMechanism extends SubsystemBase {
    private final SparkMax shooterSpark1;
    private final SparkMax shooterSpark2;
    private final SparkMax kickerSpark;

    public ShooterMechanism(
        int shooterSparkPort1,
        int shooterSparkPort2,
        int kickerSparkPort
        
    ) {
        shooterSpark1 = new SparkMax(shooterSparkPort1, MotorType.kBrushless);
        shooterSpark2 = new SparkMax(shooterSparkPort2, MotorType.kBrushless);
        kickerSpark = new SparkMax(kickerSparkPort, MotorType.kBrushless);
        configureMotor(shooterSpark1, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureMotor(shooterSpark2, IdleMode.kBrake, ShooterConstants.currentLimit);
        configureMotor(kickerSpark, IdleMode.kBrake, ShooterConstants.currentLimit);

        
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

    public void setIOSpark(double speed1, double speed2) {
        shooterSpark1.set(speed1);
        shooterSpark2.set(speed2);
        kickerSpark.set(1);
    }

    public void stopIOSpark() {
        shooterSpark1.stopMotor();
        shooterSpark2.stopMotor();
        kickerSpark.stopMotor();
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
