package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Bindings;
import frc.robot.Constants.MotorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex m_ElevatorMotor1;
    private final SparkFlex m_ElevatorMotor2;
    private final RelativeEncoder m_RelativeEncoder;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
    private PIDController m_ElevatorPIDController;
    SparkFlexConfig config = new SparkFlexConfig();

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    m_ElevatorMotor1 = new SparkFlex(MotorConstants.kElevatorMotor1CANID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkFlex(MotorConstants.kElevatorMotor2CANID, MotorType.kBrushless);
    updateMotorSettings(m_ElevatorMotor1);
    updateMotorSettings(m_ElevatorMotor2);
    m_RelativeEncoder = m_ElevatorMotor1.getEncoder();
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(1);
    zeroEncoder();

    m_ElevatorPIDController = new PIDController(0.04, 0, 0.001);
    m_ElevatorPIDController.setTolerance(1.434);

  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void goToSetpoint(double setpoint) {
    //double feedforward = 0.01;
    double speed = -m_ElevatorPIDController.calculate(getRelativeEncoderPosition(), setpoint);
    //speed = (speed>0) ? speed + feedforward : speed-feedforward;
    setSpeed(speed);
    SmartDashboard.putNumber("ELEVATOR setpoint", setpoint);
  }

  public boolean atSetpoint() {
    return m_ElevatorPIDController.atSetpoint();
  }

  public void setSpeed(double speed) {
    // positive speed goes up
    speed = (speed > MotorConstants.kElevatorMotorsMaxSpeed) ? MotorConstants.kElevatorMotorsMaxSpeed : speed;
    speed = (speed < -MotorConstants.kElevatorMotorsMaxSpeed) ? -MotorConstants.kElevatorMotorsMaxSpeed : speed;

    speed = (bottomLimitSwitch.get() && speed < 0) ? 0 : speed;

    if (getRelativeEncoderPosition() > -5 && speed<0) { //limit going down
      if (Bindings.getOperatorShiftPressed()) {
        speed*=0.75; // override but also slowing a liil
      } else {
        speed*=0.25;
      }
    } else if (getRelativeEncoderPosition()<-78.2 && speed>0){
      speed = 0; // hard limit
    } else if (getRelativeEncoderPosition()<-76.5 && speed>0) {
      speed*=0.2;
    }
    m_ElevatorMotor1.set(-speed);
    m_ElevatorMotor2.set(speed);

    SmartDashboard.putNumber("ELEVATOR speed", speed);
  }

  public void stopElevatorMotors() {
    m_ElevatorMotor1.stopMotor();
    m_ElevatorMotor2.stopMotor();
    SmartDashboard.putNumber("ELEVATOR speed", 0);
  }

  public RelativeEncoder getRelativeEncoder() {
    return m_RelativeEncoder;
  }

  public double getRelativeEncoderPosition() {
    return m_RelativeEncoder.getPosition();
  }

  public void zeroEncoder() {
    m_RelativeEncoder.setPosition(0);
  }

  public boolean getTopLimitSwitchState() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitchState() {
    return bottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ELEVATOR position", getRelativeEncoderPosition());
    SmartDashboard.putBoolean("Upper Limit Switch", getTopLimitSwitchState());
    SmartDashboard.putBoolean("Lower Limit Switch", getBottomLimitSwitchState());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
