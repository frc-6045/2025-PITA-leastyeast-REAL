package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

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
    private RelativeEncoder m_IntakeEncoder;
    private SparkClosedLoopController m_IntakePIDController;
    private double targetRPM = 0; // Track current target for error calculation

    public IntakeSubsystem() {
        m_IntakeMotor1 = new SparkFlex(MotorConstants.kIntakeMotorCANID, MotorType.kBrushless);

        updateMotorSettings(m_IntakeMotor1);
        armEncoder = m_IntakeMotor1.getAbsoluteEncoder();
        m_IntakeEncoder = m_IntakeMotor1.getEncoder();
        m_IntakePIDController = m_IntakeMotor1.getClosedLoopController();

        SmartDashboard.putNumber("offset2", 0);
        SmartDashboard.putNumber("offset3", 0);
        SmartDashboard.putNumber("offset4", 0);

        // Initialize intake velocity testing slider
        SmartDashboard.putNumber("INTAKE Test RPM", 0);

        // Initialize PID tuning values from constants
        SmartDashboard.putNumber("INTAKE PID kP", MotorConstants.kIntakeVelocityP);
        SmartDashboard.putNumber("INTAKE PID kI", MotorConstants.kIntakeVelocityI);
        SmartDashboard.putNumber("INTAKE PID kD", MotorConstants.kIntakeVelocityD);
        SmartDashboard.putNumber("INTAKE PID kFF", MotorConstants.kIntakeVelocityFF);
        SmartDashboard.putBoolean("INTAKE PID Tuning Mode", false);
    }
    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(MotorConstants.kIntakeVelocityP, MotorConstants.kIntakeVelocityI, MotorConstants.kIntakeVelocityD)
            .velocityFF(MotorConstants.kIntakeVelocityFF);
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
        targetRPM = 0;
    }

    /**
     * Sets the intake motor to a specific velocity in RPM using PID control.
     * Positive values spin the intake inward, negative values spin outward.
     * @param rpm Target velocity in rotations per minute
     */
    public void setVelocityRPM(double rpm) {
        targetRPM = MathUtil.clamp(rpm, -MotorConstants.kIntakeMaxVelocityRPM, MotorConstants.kIntakeMaxVelocityRPM);
        m_IntakePIDController.setReference(targetRPM, ControlType.kVelocity);
    }

    /**
     * Gets the current velocity of the intake motor in RPM.
     * @return Current velocity in RPM
     */
    public double getVelocityRPM() {
        return m_IntakeEncoder.getVelocity();
    }

    /**
     * Updates PID gains from SmartDashboard values if tuning mode is enabled.
     * This allows real-time tuning without redeploying code.
     */
    private void updatePIDFromDashboard() {
        boolean tuningMode = SmartDashboard.getBoolean("INTAKE PID Tuning Mode", false);

        if (tuningMode) {
            double kP = SmartDashboard.getNumber("INTAKE PID kP", MotorConstants.kIntakeVelocityP);
            double kI = SmartDashboard.getNumber("INTAKE PID kI", MotorConstants.kIntakeVelocityI);
            double kD = SmartDashboard.getNumber("INTAKE PID kD", MotorConstants.kIntakeVelocityD);
            double kFF = SmartDashboard.getNumber("INTAKE PID kFF", MotorConstants.kIntakeVelocityFF);

            // Update the PID configuration
            config.closedLoop
                .pid(kP, kI, kD)
                .velocityFF(kFF);
            m_IntakeMotor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public boolean coralDetected() {
        return getDistanceSensorOutput()<(AutoScoreConstants.coralLocation0+AutoScoreConstants.coralLocation1)/2;
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
            case 2:                return new Translation2d(0, SmartDashboard.getNumber("offset2", 0));
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
        // Update PID gains from dashboard if tuning mode is enabled
        updatePIDFromDashboard();

        // Distance sensor telemetry
        SmartDashboard.putNumber("INTAKE distance sensor", getDistanceSensorOutput());
        SmartDashboard.putNumber("INTAKE coral position", getCoralPosition());
        SmartDashboard.putBoolean("INTAKE coral detected", coralDetected());

        // Velocity telemetry for operators and PID tuning
        double actualRPM = getVelocityRPM();
        SmartDashboard.putNumber("INTAKE actual RPM", actualRPM);
        SmartDashboard.putNumber("INTAKE target RPM", targetRPM);
        SmartDashboard.putNumber("INTAKE velocity error", targetRPM - actualRPM);
        SmartDashboard.putNumber("INTAKE velocity error %",
            targetRPM != 0 ? ((targetRPM - actualRPM) / targetRPM) * 100 : 0);

        // At-speed indicator for operator feedback
        boolean atSpeed = Math.abs(targetRPM - actualRPM) < 100 && Math.abs(targetRPM) > 100;
        SmartDashboard.putBoolean("INTAKE at speed", atSpeed);
    }
}
