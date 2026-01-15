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
import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    // Hardware
    private final SparkFlex m_FlywheelMotor;
    private final RelativeEncoder m_Encoder;
    private SparkFlexConfig config = new SparkFlexConfig();

    // PID Controller (WPILib)
    private PIDController m_PIDController;

    // PID caching - store last values to avoid unnecessary setPID calls
    private double cachedP = FlywheelConstants.kDefaultP;
    private double cachedI = FlywheelConstants.kDefaultI;
    private double cachedD = FlywheelConstants.kDefaultD;
    private double cachedTolerance = FlywheelConstants.kDefaultToleranceRPM;
    private double cachedFF = FlywheelConstants.kDefaultFF;

    // Target RPM (set by command or dashboard)
    private double targetRPM = 0.0;

    // Output tracking for telemetry
    private double lastPIDOutput = 0.0;
    private double lastFFOutput = 0.0;

    public FlywheelSubsystem() {
        m_FlywheelMotor = new SparkFlex(FlywheelConstants.kFlywheelMotorCANID, MotorType.kBrushless);
        m_Encoder = m_FlywheelMotor.getEncoder();

        updateMotorSettings(m_FlywheelMotor);

        // Initialize WPILib PID controller
        m_PIDController = new PIDController(
            FlywheelConstants.kDefaultP,
            FlywheelConstants.kDefaultI,
            FlywheelConstants.kDefaultD
        );
        m_PIDController.setTolerance(FlywheelConstants.kDefaultToleranceRPM);

        // Initialize dashboard tunable values
        initDashboard();
    }

    private void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kCoast)  // Coast for flywheel - lets it spin down naturally
            .smartCurrentLimit(FlywheelConstants.kFlywheelMotorCurrentLimit);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void initDashboard() {
        // Initialize tunable values on SmartDashboard/Elastic
        SmartDashboard.putNumber("FLYWHEEL Target RPM", FlywheelConstants.kDefaultTargetRPM);
        SmartDashboard.putNumber("FLYWHEEL P", FlywheelConstants.kDefaultP);
        SmartDashboard.putNumber("FLYWHEEL I", FlywheelConstants.kDefaultI);
        SmartDashboard.putNumber("FLYWHEEL D", FlywheelConstants.kDefaultD);
        SmartDashboard.putNumber("FLYWHEEL FF", FlywheelConstants.kDefaultFF);
        SmartDashboard.putNumber("FLYWHEEL Tolerance", FlywheelConstants.kDefaultToleranceRPM);
    }

    /**
     * Update PID gains from dashboard if they have changed.
     * Uses caching to avoid setting gains every cycle.
     */
    private void updatePIDFromDashboard() {
        double newP = SmartDashboard.getNumber("FLYWHEEL P", cachedP);
        double newI = SmartDashboard.getNumber("FLYWHEEL I", cachedI);
        double newD = SmartDashboard.getNumber("FLYWHEEL D", cachedD);
        double newTolerance = SmartDashboard.getNumber("FLYWHEEL Tolerance", cachedTolerance);
        double newFF = SmartDashboard.getNumber("FLYWHEEL FF", cachedFF);

        // Only update if values have actually changed
        if (newP != cachedP || newI != cachedI || newD != cachedD) {
            m_PIDController.setPID(newP, newI, newD);
            cachedP = newP;
            cachedI = newI;
            cachedD = newD;
        }

        if (newTolerance != cachedTolerance) {
            m_PIDController.setTolerance(newTolerance);
            cachedTolerance = newTolerance;
        }

        if (newFF != cachedFF) {
            cachedFF = newFF;
        }
    }

    /**
     * Get the target RPM from the dashboard.
     * @return target RPM value from dashboard
     */
    public double getTargetRPMFromDashboard() {
        return SmartDashboard.getNumber("FLYWHEEL Target RPM", FlywheelConstants.kDefaultTargetRPM);
    }

    /**
     * Run flywheel to reach target RPM using PID + feedforward.
     * @param targetRPM the desired RPM
     */
    public void runToRPM(double targetRPM) {
        this.targetRPM = targetRPM;

        // Update PID from dashboard (with caching)
        updatePIDFromDashboard();

        double currentRPM = getVelocityRPM();

        // Calculate PID output
        lastPIDOutput = m_PIDController.calculate(currentRPM, targetRPM);

        // Calculate feedforward: FF * targetRPM
        // This provides a baseline output proportional to desired velocity
        lastFFOutput = cachedFF * targetRPM;

        // Combine PID + Feedforward
        double output = lastPIDOutput + lastFFOutput;

        // Clamp to 0-1 (forward only)
        output = MathUtil.clamp(output, 0.0, 1.0);

        m_FlywheelMotor.set(output);

        SmartDashboard.putNumber("FLYWHEEL output", output);
    }

    /**
     * Stop the flywheel motor.
     */
    public void stop() {
        m_FlywheelMotor.set(0);
        targetRPM = 0;
        m_PIDController.reset();  // Reset PID integrator
        SmartDashboard.putNumber("FLYWHEEL output", 0);
    }

    /**
     * Get current flywheel velocity in RPM.
     * @return velocity in RPM
     */
    public double getVelocityRPM() {
        return m_Encoder.getVelocity();
    }

    /**
     * Check if flywheel is at the target setpoint within tolerance.
     * @return true if at setpoint
     */
    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    /**
     * Get the current target RPM.
     * @return current target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    @Override
    public void periodic() {
        // Publish telemetry
        SmartDashboard.putNumber("FLYWHEEL Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("FLYWHEEL Target RPM Current", targetRPM);
        SmartDashboard.putBoolean("FLYWHEEL At Setpoint", atSetpoint());
        SmartDashboard.putNumber("FLYWHEEL PID Output", lastPIDOutput);
        SmartDashboard.putNumber("FLYWHEEL FF Output", lastFFOutput);
    }

    @Override
    public void simulationPeriodic() {
        // Simulation support if needed
    }
}
