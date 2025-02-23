package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.SparkFlex;
//import com.revrobotics.CANSparkLowLevel.MotorType;

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
    m_ElevatorMotor1 = new SparkFlex(MotorConstants.kSparkFlexElevatorMotor1CANID, MotorType.kBrushless);
    m_ElevatorMotor2 = new SparkFlex(MotorConstants.kSparkFlexElevatorMotor2CANID, MotorType.kBrushless);
    updateMotorSettings(m_ElevatorMotor1);
    updateMotorSettings(m_ElevatorMotor2);
    m_RelativeEncoder = m_ElevatorMotor1.getEncoder();
    topLimitSwitch = new DigitalInput(2);
    bottomLimitSwitch = new DigitalInput(1);

    m_ElevatorPIDController = new PIDController(0.04, 0, 0.001);
    //m_ElevatorPIDController.enableContinuousInput(0, 1);
    m_ElevatorPIDController.setTolerance(1.434);

  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void goToSetpoint(double setpoint) {
    //double feedforward = 0.01;
        //if (m_ArmMotor.getAbsoluteEncoderPosition()-setPoint<0.01 && m_ArmMotor.getAbsoluteEncoderPosition()-setPoint>-0.01) m_ArmMotor.stopArmMotor();;;
        double speed = -m_ElevatorPIDController.calculate(getRelativeEncoderPosition(), setpoint);
        //speed = (speed>0) ? speed + feedforward : speed-feedforward;
        setSpeed(speed);
        System.out.println("PIDElevator output (speed): " + speed + "\nset point: " + m_ElevatorPIDController.getSetpoint() + "\ncurrent position: " + getRelativeEncoderPosition());
  }

  public boolean atSetpoint() {
    return m_ElevatorPIDController.atSetpoint();
  }

  public void setSpeed(double speed) {
    speed = (speed > MotorConstants.kSparkFlexElevatorMotorsMaxSpeed) ? MotorConstants.kSparkFlexElevatorMotorsMaxSpeed : speed;
    speed = (speed < -MotorConstants.kSparkFlexElevatorMotorsMaxSpeed) ? -MotorConstants.kSparkFlexElevatorMotorsMaxSpeed : speed;
    //System.out.println("speed: " + speed);
   //
   
   
   
   
   speed = ((topLimitSwitch.get() && speed > 0) || (bottomLimitSwitch.get() && speed < 0)) ? 0 : speed;
    //System.out.println("speed: " + speed + "\n" + bottomLimitSwitch.get() + " "  + topLimitSwitch.get());

    m_ElevatorMotor1.set(-speed);
    m_ElevatorMotor2.set(speed);
  }

  public void stopElevatorMotors() {
    m_ElevatorMotor1.stopMotor();
    m_ElevatorMotor2.stopMotor();
  }

  public RelativeEncoder getRelativeEncoder() {
    return m_RelativeEncoder;
  }

  public double getRelativeEncoderPosition() {
    //System.out.println("position is " + m_RelativeEncoder.getPosition());
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
    SmartDashboard.putNumber("Elevator", getRelativeEncoderPosition());
    if (bottomLimitSwitch.get()) {zeroEncoder();}
    SmartDashboard.putBoolean("Upper", getTopLimitSwitchState());
    SmartDashboard.putBoolean("lower", getBottomLimitSwitchState());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
