package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoScoreConstants;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex m_IntakeMotor1;
    private SparkFlexConfig config = new SparkFlexConfig();
    private AnalogPotentiometer m_DistanceSensor = new AnalogPotentiometer(3);
    public AbsoluteEncoder armEncoder;

    public IntakeSubsystem() {
        m_IntakeMotor1 = new SparkFlex(MotorConstants.kIntakeMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_IntakeMotor1);
        armEncoder = m_IntakeMotor1.getAbsoluteEncoder();
        SmartDashboard.putNumber("offset2", 0);
        SmartDashboard.putNumber("offset3", 0);
        SmartDashboard.putNumber("offset4", 0);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getDistanceSensorOutput() {
        return m_DistanceSensor.get();
    }

    public double getDistanceInches() {
        return (m_DistanceSensor.get()-0.024)/(7.76*0.001)*1.267;
    }

    public void setSpeed(double speed) {
        speed = -MathUtil.clamp(speed, -MotorConstants.kIntakeMotorMaxSpeed, MotorConstants.kIntakeMotorMaxSpeed);
        m_IntakeMotor1.set(speed);

        SmartDashboard.putNumber("INTAKE speed", speed);
    }

    public void stopIntake() {
        m_IntakeMotor1.set(0);
    }

    public boolean coralDetected() {
        return getDistanceSensorOutput()<0.085;
    }
    
    /**
     * 0 no coral, 1 is closest to elev, 4 is farthest from elev
     * @return coral position
     */
    public int getCoralPosition() {
        if (getDistanceSensorOutput()>(AutoScoreConstants.coralLocation0+AutoScoreConstants.coralLocation1)/2)
            return 0;
        else if (getDistanceSensorOutput()>(AutoScoreConstants.coralLocation1+AutoScoreConstants.coralLocation2)/2)
            return 1;
        else if (getDistanceSensorOutput()>(AutoScoreConstants.coralLocation2+AutoScoreConstants.coralLocation3)/2)
            return 2;
        else if (getDistanceSensorOutput()>(AutoScoreConstants.coralLocation3+AutoScoreConstants.coralLocation4)/2)
            return 3;
        else if (getDistanceSensorOutput()>(AutoScoreConstants.coralLocation4+0)/2)
            return 4;

        return 0;
    }

    public Translation2d getAlignOffset() {
        switch (getCoralPosition()) {
            case 0:
                System.out.println("there's no coral :(");

                return new Translation2d();
            case 1:
                return AutoScoreConstants.autoScoreCoralOffset1;
            case 2:
                return new Translation2d(0, SmartDashboard.getNumber("offset2", 0));
            case 3:
                return new Translation2d(0, SmartDashboard.getNumber("offset3", 0));
            case 4:
                return new Translation2d(0, SmartDashboard.getNumber("offset4", 0));
        }

        return new Translation2d();
    }

    public double getAlignOffsetLimelightTX() {
        switch (getCoralPosition()) {
            case 0:
                System.out.println("there's no coral :(");

                return 0;
            case 1:
                return 0;
            case 2:
                return -4.5;
            case 3:
                return -9;
            case 4:
                return -13;
        }

        return 0.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("INTAKE distance sensor", getDistanceSensorOutput());
        SmartDashboard.putNumber("INTAKE coral position", getCoralPosition());
        SmartDashboard.putBoolean("INTAKE coral detected", coralDetected());
    }
}
