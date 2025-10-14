// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex m_ArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  SparkFlexConfig config = new SparkFlexConfig();
  PIDController m_ArmPIDController = new PIDController(9, 0, 0);

  /** Creates a new ArmSubsystem that controls the robot's arm mechanism. */
  public ArmSubsystem(IntakeSubsystem intake) {
    m_ArmMotor = new SparkFlex(MotorConstants.kArmMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = intake.armEncoder;
    m_ArmPIDController.setTolerance(0.01);

    updateMotorSettings(m_ArmMotor);
    m_ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kArmMotorCurrentLimit); // FIXED: Was using kIntakeMotorCurrentLimit
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  /**
   * Commands the arm to move to a target setpoint using PID control.
   *
   * @param setpoint The target position in the sketchy offset coordinate system (0-1 range)
   * @param speedLimit Maximum speed for the arm motor (0-1 range)
   */
  public void goToSetpoint(double setpoint, double speedLimit) {
    SmartDashboard.putNumber("ARM setpoint (sketchy btw)", getSketchyOffsettedPosition());
    SmartDashboard.putNumber("ARM difference", setpoint-getSketchyOffsettedPosition());

    // Wrap setpoint to 0-1 range for proper modulo arithmetic with encoder
    double wrappedSetpoint = (setpoint + MotorConstants.kArmEncoderPositionWrapOffset) % 1;
    double speed = m_ArmPIDController.calculate(getSketchyOffsettedPosition(), wrappedSetpoint);

    setSpeed(MathUtil.clamp(speed, -speedLimit, speedLimit));
  }

  /**
   * Commands the arm to move to a target setpoint using default speed limit.
   *
   * @param setpoint The target position in the sketchy offset coordinate system (0-1 range)
   */
  public void goToSetpoint(double setpoint) {
    goToSetpoint(setpoint, MotorConstants.kArmMotorSetpointMaxSpeed);
  }

  public boolean atSetpoint() {
    return m_ArmPIDController.atSetpoint();
  }

  /**
   * Sets the arm motor speed with safety limits to prevent turnbuckle damage.
   *
   * @param speed Desired motor speed (-1 to 1)
   */
  public void setSpeed(double speed) {
    // Clamp to max speed
    speed = MathUtil.clamp(speed, -MotorConstants.kArmMotorMaxSpeed, MotorConstants.kArmMotorMaxSpeed);

    // Prevent turnbuckle from being run over - software limits
    double encoderPos = getAbsoluteEncoderPosition();

    if (speed < 0 && encoderPos < PositionConstants.kArmLimit2 && encoderPos > PositionConstants.kMiddleOfArmLimit) {
      speed = 0;
      DriverStation.reportWarning("ARM: Hit software limit 2 (preventing turnbuckle collision)", false);
    }
    if (speed > 0 && encoderPos > PositionConstants.kArmLimit1 && encoderPos < PositionConstants.kMiddleOfArmLimit) {
      speed = 0;
      DriverStation.reportWarning("ARM: Hit software limit 1 (preventing turnbuckle collision)", false);
    }

    m_ArmMotor.set(speed);
    SmartDashboard.putNumber("ARM speed", speed);
  }

  public void stopArmMotor() {
    m_ArmMotor.stopMotor();
    SmartDashboard.putNumber("ARM speed", 0);
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return m_AbsoluteEncoder;
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }

  /**
   * Gets the arm position with the sketchy offset applied.
   * This offset shifts the coordinate system to make setpoints more intuitive.
   *
   * @return The offsetted position in 0-1 range
   */
  public double getSketchyOffsettedPosition() {
    return (m_AbsoluteEncoder.getPosition() + PositionConstants.kSketchyOffset + MotorConstants.kArmEncoderPositionWrapOffset) % 1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM position (SKETCHY)", (getAbsoluteEncoderPosition()+PositionConstants.kSketchyOffset)%1);
    SmartDashboard.putNumber("raw ARM position", getAbsoluteEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
