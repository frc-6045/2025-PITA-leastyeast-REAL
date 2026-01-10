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

    // Cached PID values to avoid updating every cycle
    private double m_cachedP = MotorConstants.kFlywheelP;
    private double m_cachedI = MotorConstants.kFlywheelI;
    private double m_cachedD = MotorConstants.kFlywheelD;
    private double m_cachedTolerance = MotorConstants.kFlywheelTolerance;
    private double m_cachedFF = MotorConstants.kFlywheelFF;

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
        SmartDashboard.putNumber("Flywheel FF", MotorConstants.kFlywheelFF);
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
     * Update PID and FF values from SmartDashboard (only when values change)
     */
    public void updatePIDFromDashboard() {
        double p = SmartDashboard.getNumber("Flywheel P", MotorConstants.kFlywheelP);
        double i = SmartDashboard.getNumber("Flywheel I", MotorConstants.kFlywheelI);
        double d = SmartDashboard.getNumber("Flywheel D", MotorConstants.kFlywheelD);
        double tolerance = SmartDashboard.getNumber("Flywheel Tolerance", MotorConstants.kFlywheelTolerance);
        double ff = SmartDashboard.getNumber("Flywheel FF", MotorConstants.kFlywheelFF);

        // Only update PID if values changed
        if (p != m_cachedP || i != m_cachedI || d != m_cachedD) {
            m_PIDController.setPID(p, i, d);
            m_cachedP = p;
            m_cachedI = i;
            m_cachedD = d;
        }

        if (tolerance != m_cachedTolerance) {
            m_PIDController.setTolerance(tolerance);
            m_cachedTolerance = tolerance;
        }

        m_cachedFF = ff;
    }

    /**
     * Run the flywheel to the target RPM using feedforward + PID control
     */
    public void runToTargetRPM() {
        m_targetRPM = getTargetRPM();
        updatePIDFromDashboard();

        // Calculate feedforward: baseline power to maintain target RPM
        double feedforward = (m_targetRPM / MotorConstants.kFlywheelMaxRPM) * m_cachedFF * 1000;

        // Calculate PID correction
        double pidOutput = m_PIDController.calculate(getCurrentRPM(), m_targetRPM);

        // Combine feedforward + PID
        double output = feedforward + pidOutput;

        // Clamp output to valid motor range [0, 1] for forward-only flywheel
        output = MathUtil.clamp(output, 0, 1);

        m_FlywheelMotor.set(output);
        SmartDashboard.putNumber("Flywheel Output", output);
        SmartDashboard.putNumber("Flywheel FF Output", feedforward);
        SmartDashboard.putNumber("Flywheel PID Output", pidOutput);
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
