package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public void setSpeed(double percentMotor1, double percentMotor2) {
        double speed1 = MotorConstants.kIntakeMotorsSpeed * percentMotor1;
        double speed2 = MotorConstants.kIntakeMotorsSpeed * percentMotor2;
        speed1= MathUtil.clamp(speed1, -MotorConstants.kIntakeMotorsMaxSpeed, MotorConstants.kIntakeMotorsMaxSpeed);
        speed2= MathUtil.clamp(speed2, -MotorConstants.kIntakeMotorsMaxSpeed, MotorConstants.kIntakeMotorsMaxSpeed);
        m_IntakeMotor1.set(speed1);
        m_IntakeMotor2.set(speed2);
        //m_AlgaeMotor.set();

        double[] arr = {speed1, speed2};
        SmartDashboard.putNumberArray("INTAKE speeds", arr);
    }

    public void stopIntake() {
        m_IntakeMotor1.set(0);
        m_IntakeMotor2.set(0);
        //m_AlgaeMotor.set(0);
    }
}
