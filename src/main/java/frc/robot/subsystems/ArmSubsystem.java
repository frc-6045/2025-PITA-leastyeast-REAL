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
//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex m_ArmMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  SparkFlexConfig config = new SparkFlexConfig();
  //private final ArmFeedforward m_ArmFeedforward = new ArmFeedforward(0, 0, 0);
  PIDController m_ArmPIDController = new PIDController(3.5, 0, 1);
  Elastic.Notification notification = new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "arm encoder not plugged in!??!?!!??!?", "blame build team");


  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {
    m_ArmMotor = new SparkFlex(MotorConstants.kArmMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = m_ArmMotor.getAbsoluteEncoder();
    //m_ArmPIDController.enableContinuousInput(0, 1);
    m_ArmPIDController.setTolerance(0.01);

    updateMotorSettings(m_ArmMotor);
    m_ArmMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void goToSetpoint(double setpoint, double speedLimit) {
    SmartDashboard.putNumber("ARM setpoint", setpoint+PositionConstants.kSketchyOffset);
    SmartDashboard.putNumber("ARM difference", setpoint-getAbsoluteEncoderPosition());
    double speed = m_ArmPIDController.calculate(
      (getAbsoluteEncoderPosition()+14+PositionConstants.kSketchyOffset)%1, 
      (setpoint+14+PositionConstants.kSketchyOffset)%1);
    //speed = (speed>0) ? speed + feedforward : speed-feedforward;
    setSpeed(MathUtil.clamp(speed, -speedLimit, speedLimit));
    //System.out.println("PIDArm output (speed): " + speed + "\nset point: " + m_ArmPIDController.getSetpoint() + "\ncurrent position: " + getAbsoluteEncoderPosition());
  }

  public void goToSetpoint(double setpoint) {
    SmartDashboard.putNumber("ARM setpoint", setpoint+PositionConstants.kSketchyOffset);
    SmartDashboard.putNumber("ARM difference", setpoint-getAbsoluteEncoderPosition());
    double speed = m_ArmPIDController.calculate(
      (getAbsoluteEncoderPosition()+14+PositionConstants.kSketchyOffset)%1, 
      (setpoint+14)%1);
    setSpeed(MathUtil.clamp(speed, -MotorConstants.kArmMotorSetpointMaxSpeed, MotorConstants.kArmMotorSetpointMaxSpeed));
  }

  public boolean atSetpoint() {
    return m_ArmPIDController.atSetpoint();
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kArmMotorMaxSpeed)
      speed = MotorConstants.kArmMotorMaxSpeed;
    if (speed<-MotorConstants.kArmMotorMaxSpeed)
      speed = -MotorConstants.kArmMotorMaxSpeed;
    
    // prevent turnbuckle from being run over
    
    if (speed<0 && 
    (getAbsoluteEncoderPosition()<PositionConstants.kArmLimit2 && 
    getAbsoluteEncoderPosition()>PositionConstants.kMiddleOfArmLimit)) {
      speed = 0;
      System.out.println("LIMIT 2");
    }
    if (speed>0 &&
    (getAbsoluteEncoderPosition()>PositionConstants.kArmLimit1) &&
    getAbsoluteEncoderPosition()<PositionConstants.kMiddleOfArmLimit) {
      speed=0;
      System.out.println("LIMIT 1");
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ARM position (SKETCHY)", (getAbsoluteEncoderPosition()+PositionConstants.kSketchyOffset)%1);
    SmartDashboard.putNumber("raw ARM position", getAbsoluteEncoderPosition());
    // if (getAbsoluteEncoderPosition()==0) {
    //   Elastic.sendNotification(notification);
    //   System.out.println("blame build for not plugging in encoder");

    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
