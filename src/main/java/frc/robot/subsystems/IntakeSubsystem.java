package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_IntakeMotor1;
    private final SparkFlex m_IntakeMotor2;
    //private final SparkFlex m_AlgaeMotor;
    SparkFlexConfig config = new SparkFlexConfig();

    public IntakeSubsystem() {
        m_IntakeMotor1 = new SparkFlex(MotorConstants.kIntakeMotor1CANID, MotorType.kBrushless);
        m_IntakeMotor2 = new SparkFlex(MotorConstants.kIntakeMotor2CANID, MotorType.kBrushless);
        //m_AlgaeMotor = new SparkFlex(MotorConstants.kAlgaeRemovingMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_IntakeMotor1);
        updateMotorSettings(m_IntakeMotor2);
        //updateMotorSettings(m_AlgaeMotor);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   
    public void setSpeed(double speed, double runMotor1, double runMotor2) {
        if (speed>MotorConstants.kIntakeMotorsMaxSpeed || speed<-MotorConstants.kIntakeMotorsMaxSpeed)
          speed = MotorConstants.kIntakeMotorsMaxSpeed;
        m_IntakeMotor1.set(speed*runMotor1);
        m_IntakeMotor2.set(speed*runMotor2);
        //m_AlgaeMotor.set(speed);
    }
}
