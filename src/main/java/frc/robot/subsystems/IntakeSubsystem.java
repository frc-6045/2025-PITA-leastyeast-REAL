package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_IntakeMotor1;
    SparkFlexConfig config = new SparkFlexConfig();
    AnalogPotentiometer m_DistanceSensor = new AnalogPotentiometer(3);

    public IntakeSubsystem() {
        m_IntakeMotor1 = new SparkFlex(MotorConstants.kIntakeMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_IntakeMotor1);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getDistanceSensorOutput() {
        return m_DistanceSensor.get();
    }

    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaxSpeed, MotorConstants.kIntakeMotorMaxSpeed);
        m_IntakeMotor1.set(speed);

        SmartDashboard.putNumber("INTAKE speed", speed);
    }

    public void stopIntake() {
        m_IntakeMotor1.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("INTAKE distance sensor", getDistanceSensorOutput());
    }
}
