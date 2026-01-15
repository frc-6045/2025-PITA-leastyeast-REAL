package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FlywheelConstants;

public class FlywheelSubsystem extends SubsystemBase {
    private static final double EPSILON = 1e-9;

    private final SparkFlex m_FlywheelMotor;
    private final RelativeEncoder m_Encoder;
    private final PIDController m_PIDController;
    SparkFlexConfig config = new SparkFlexConfig();

    // Cached PID values to avoid unnecessary updates
    private double cachedP = FlywheelConstants.kDefaultP;
    private double cachedI = FlywheelConstants.kDefaultI;
    private double cachedD = FlywheelConstants.kDefaultD;
    private double cachedTolerance = FlywheelConstants.kDefaultToleranceRPM;
    private double cachedFF = FlywheelConstants.kDefaultFF;

    private double targetRPM = 0.0;
    private double lastPIDOutput = 0.0;
    private double lastFFOutput = 0.0;

    public FlywheelSubsystem() {
        m_FlywheelMotor = new SparkFlex(FlywheelConstants.kFlywheelMotorCANID, MotorType.kBrushless);
        m_Encoder = m_FlywheelMotor.getEncoder();
        m_PIDController = new PIDController(
            FlywheelConstants.kDefaultP,
            FlywheelConstants.kDefaultI,
            FlywheelConstants.kDefaultD
        );
        m_PIDController.setTolerance(FlywheelConstants.kDefaultToleranceRPM);

        updateMotorSettings(m_FlywheelMotor);
        initDashboard();
    }

    public void updateMotorSettings(SparkFlex motor) {
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(FlywheelConstants.kFlywheelMotorCurrentLimit)
            .inverted(false);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void initDashboard() {
        SmartDashboard.putNumber("FLYWHEEL Target RPM", FlywheelConstants.kDefaultTargetRPM);
        SmartDashboard.putNumber("FLYWHEEL P", FlywheelConstants.kDefaultP);
        SmartDashboard.putNumber("FLYWHEEL I", FlywheelConstants.kDefaultI);
        SmartDashboard.putNumber("FLYWHEEL D", FlywheelConstants.kDefaultD);
        SmartDashboard.putNumber("FLYWHEEL FF", FlywheelConstants.kDefaultFF);
        SmartDashboard.putNumber("FLYWHEEL Tolerance", FlywheelConstants.kDefaultToleranceRPM);
    }

    private void updatePIDFromDashboard() {
        double newP = SmartDashboard.getNumber("FLYWHEEL P", cachedP);
        double newI = SmartDashboard.getNumber("FLYWHEEL I", cachedI);
        double newD = SmartDashboard.getNumber("FLYWHEEL D", cachedD);
        double newTolerance = SmartDashboard.getNumber("FLYWHEEL Tolerance", cachedTolerance);
        double newFF = SmartDashboard.getNumber("FLYWHEEL FF", cachedFF);

        boolean pidChanged = Math.abs(newP - cachedP) > EPSILON
            || Math.abs(newI - cachedI) > EPSILON
            || Math.abs(newD - cachedD) > EPSILON;

        if (pidChanged) {
            m_PIDController.setPID(newP, newI, newD);
            cachedP = newP;
            cachedI = newI;
            cachedD = newD;
        }

        if (Math.abs(newTolerance - cachedTolerance) > EPSILON) {
            m_PIDController.setTolerance(newTolerance);
            cachedTolerance = newTolerance;
        }

        if (Math.abs(newFF - cachedFF) > EPSILON) {
            cachedFF = newFF;
        }
    }

    public double getTargetRPMFromDashboard() {
        return SmartDashboard.getNumber("FLYWHEEL Target RPM", FlywheelConstants.kDefaultTargetRPM);
    }

    public void runToRPM(double rpm) {
        this.targetRPM = MathUtil.clamp(rpm, 0, FlywheelConstants.kMaxRPM);
        updatePIDFromDashboard();

        double currentRPM = getVelocityRPM();
        lastPIDOutput = m_PIDController.calculate(currentRPM, this.targetRPM);
        lastFFOutput = cachedFF * this.targetRPM;

        double output = MathUtil.clamp(lastPIDOutput + lastFFOutput, 0.0, 1.0);
        m_FlywheelMotor.set(output);

        SmartDashboard.putNumber("FLYWHEEL output", output);
    }

    public void stop() {
        m_FlywheelMotor.set(0);
        targetRPM = 0;
        m_PIDController.reset();
        SmartDashboard.putNumber("FLYWHEEL output", 0);
    }

    public double getVelocityRPM() {
        return m_Encoder.getVelocity();
    }

    public boolean atSetpoint() {
        return m_PIDController.atSetpoint();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FLYWHEEL Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("FLYWHEEL Target RPM Current", targetRPM);
        SmartDashboard.putBoolean("FLYWHEEL At Setpoint", atSetpoint());
        SmartDashboard.putNumber("FLYWHEEL PID Output", lastPIDOutput);
        SmartDashboard.putNumber("FLYWHEEL FF Output", lastFFOutput);
    }

    @Override
    public void simulationPeriodic() {}

    public void close() {
        m_FlywheelMotor.close();
        m_PIDController.close();
    }
}
