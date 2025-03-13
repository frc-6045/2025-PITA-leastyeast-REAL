package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

//import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_IntakeMotor1;
    private SparkFlexConfig config = new SparkFlexConfig();
    private static AnalogPotentiometer m_DistanceSensor = new AnalogPotentiometer(3);

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

    public static double getDistanceSensorOutput() {
        return m_DistanceSensor.get();
    }

    public double getDistanceInches() {
        return (m_DistanceSensor.get()-0.024)/(7.76*0.001)*1.267;
    }

    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaxSpeed, MotorConstants.kIntakeMotorMaxSpeed);
        m_IntakeMotor1.set(speed);

        SmartDashboard.putNumber("INTAKE speed", speed);
    }

    public void stopIntake() {
        m_IntakeMotor1.set(0);
    }

    public static boolean coralDetected() {
        return getDistanceSensorOutput()<0.085;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("INTAKE distance sensor", getDistanceSensorOutput());
        SmartDashboard.putNumber("INTAKE distance sensor inches", getDistanceInches());
        SmartDashboard.putBoolean("INTAKE coral detected", coralDetected());
        // Shuffleboard.getTab("test")
        //     .add("distance", getDistanceSensorOutput())
        //     .withWidget(BuiltInWidgets.kNumberSlider)
        //     .withProperties(Map.of("min", 0, "max", 0.15))
        //     .getEntry();
    }
}
