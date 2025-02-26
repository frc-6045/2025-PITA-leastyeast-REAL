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

public class AlgaeRemovingSubsystem extends SubsystemBase {
    private final SparkFlex m_AlgaeMotor;
    SparkFlexConfig config = new SparkFlexConfig();

    public AlgaeRemovingSubsystem() {
        m_AlgaeMotor = new SparkFlex(MotorConstants.kAlgaeRemovingMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_AlgaeMotor);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   

    public void setSpeed(double speed) {
        speed= MathUtil.clamp(speed, -MotorConstants.kAlgaeRemovingMotorMaxSpeed, MotorConstants.kAlgaeRemovingMotorMaxSpeed);

        m_AlgaeMotor.set(speed);

        SmartDashboard.putNumber("ALGAE speed", speed);
    }

    public void stopMotor() {
        m_AlgaeMotor.set(0);
        SmartDashboard.putNumber("ALGAE speed", 0);
    }
}
