package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private final SparkFlex m_FlywheelMotor;
    private final RelativeEncoder m_Encoder;
    private final SparkFlexConfig config = new SparkFlexConfig();
    private PIDController m_PIDController;

    private double m_targetRPM = 0;

    public FlywheelSubsystem() {
        m_FlywheelMotor = new SparkFlex(MotorConstants.kFlywheelMotorCANID, MotorType.kBrushless);
        m_Encoder = m_FlywheelMotor.getEncoder();

        updateMotorSettings(m_FlywheelMotor);

        // Initialize PID controller with default values
        m_PIDController = new PIDController(
            MotorConstants.kFlywheelP,
            MotorConstants.kFlywheelI,
            MotorConstants.kFlywheelD
        );
        m_PIDController.setTolerance(MotorConstants.kFlywheelTolerance);

        // Initialize SmartDashboard values for tuning
        SmartDashboard.putNumber("Flywheel Target RPM", MotorConstants.kFlywheelDefaultTargetRPM);
        SmartDashboard.putNumber("Flywheel P", MotorConstants.kFlywheelP);
        SmartDashboard.putNumber("Flywheel I", MotorConstants.kFlywheelI);
        SmartDashboard.putNumber("Flywheel D", MotorConstants.kFlywheelD);
        SmartDashboard.putNumber("Flywheel Tolerance", MotorConstants.kFlywheelTolerance);
    }

    private void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kCoast)  // Coast mode for flywheel to protect motor
            .smartCurrentLimit(MotorConstants.kFlywheelMotorCurrentLimit);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Get the target RPM from SmartDashboard
     * @return target RPM (clamped to valid range)
     */
    public double getTargetRPM() {
        double target = SmartDashboard.getNumber("Flywheel Target RPM", MotorConstants.kFlywheelDefaultTargetRPM);
        return MathUtil.clamp(target, 0, MotorConstants.kFlywheelMaxRPM);
    }

    /**
     * Get the current flywheel velocity in RPM
     * @return current RPM from encoder
     */
    public double getCurrentRPM() {
        return m_Encoder.getVelocity();
    }

    /**
     * Update PID values from SmartDashboard
     */
    public void updatePIDFromDashboard() {
        double p = SmartDashboard.getNumber("Flywheel P", MotorConstants.kFlywheelP);
        double i = SmartDashboard.getNumber("Flywheel I", MotorConstants.kFlywheelI);
        double d = SmartDashboard.getNumber("Flywheel D", MotorConstants.kFlywheelD);
        double tolerance = SmartDashboard.getNumber("Flywheel Tolerance", MotorConstants.kFlywheelTolerance);

        m_PIDController.setPID(p, i, d);
        m_PIDController.setTolerance(tolerance);
    }

    /**
     * Run the flywheel to the target RPM using PID control
     */
    public void runToTargetRPM() {
        m_targetRPM = getTargetRPM();
        updatePIDFromDashboard();

        double output = m_PIDController.calculate(getCurrentRPM(), m_targetRPM);

        // Clamp output to valid motor range [0, 1] for forward-only flywheel
        output = MathUtil.clamp(output, 0, 1);

        m_FlywheelMotor.set(output);
        SmartDashboard.putNumber("Flywheel Output", output);
    }

    /**
     * Set motor speed directly (for open-loop control if needed)
     * @param speed motor speed [-1, 1]
     */
    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        m_FlywheelMotor.set(speed);
    }

    /**
     * Stop the flywheel motor
     */
    public void stopFlywheel() {
        m_FlywheelMotor.stopMotor();
        m_PIDController.reset();
    }

    /**
     * Check if flywheel is at the target RPM within tolerance
     * @return true if at setpoint
     */
    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    /**
     * Get the current target RPM being used
     * @return target RPM
     */
    public double getActiveTargetRPM() {
        return m_targetRPM;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Actual RPM", getCurrentRPM());
        SmartDashboard.putBoolean("Flywheel At Setpoint", atSetpoint());
        SmartDashboard.putNumber("Flywheel Active Target", m_targetRPM);
    }

    @Override
    public void simulationPeriodic() {
    }
}
