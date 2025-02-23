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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex m_ArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  SparkFlexConfig config = new SparkFlexConfig();
  //private final ArmFeedforward m_ArmFeedforward = new ArmFeedforward(0, 0, 0);
  PIDController m_ArmPIDController = new PIDController(9, 0, 0);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_ArmMotor = new SparkFlex(MotorConstants.kSparkFlexArmMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = m_ArmMotor.getAbsoluteEncoder();
    m_ArmPIDController.enableContinuousInput(0, 1);
    m_ArmPIDController.setTolerance(0.01);

    updateMotorSettings(m_ArmMotor);
    m_ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void goToSetpoint(double setpoint) {
    SmartDashboard.putNumber("setpoint", setpoint);
    SmartDashboard.putNumber("difference", setpoint-getAbsoluteEncoderPosition());
    double speed = m_ArmPIDController.calculate(getAbsoluteEncoderPosition(), setpoint);
    //speed = (speed>0) ? speed + feedforward : speed-feedforward;
    setSpeed(speed);
    System.out.println("PIDArm output (speed): " + speed + "\nset point: " + m_ArmPIDController.getSetpoint() + "\ncurrent position: " + getAbsoluteEncoderPosition());
  }

  public boolean atSetpoint() {
    return m_ArmPIDController.atSetpoint();
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kSparkFlexArmMotorMaxSpeed)
      speed = MotorConstants.kSparkFlexArmMotorMaxSpeed;
    if (speed<-MotorConstants.kSparkFlexArmMotorMaxSpeed)
      speed = -MotorConstants.kSparkFlexArmMotorMaxSpeed;
    m_ArmMotor.set(speed);
    SmartDashboard.putNumber("ARM speed", speed);
  }

  public void stopArmMotor() {
    m_ArmMotor.stopMotor();
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return m_AbsoluteEncoder;
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
