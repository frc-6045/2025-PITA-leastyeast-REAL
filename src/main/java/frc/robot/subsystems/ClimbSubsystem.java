//make subsystem

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkFlex m_ClimbMotor;
  private final RelativeEncoder m_RelativeEncoder;
  SparkFlexConfig config = new SparkFlexConfig();

  public ClimbSubsystem() {
    m_ClimbMotor = new SparkFlex(MotorConstants.kClimbMotorCANID, MotorType.kBrushless);
    m_RelativeEncoder = m_ClimbMotor.getEncoder();

    updateMotorSettings(m_ClimbMotor);
    m_ClimbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kClimbMotorMaximumSpeed)
      speed = MotorConstants.kClimbMotorMaximumSpeed;
    if (speed<-MotorConstants.kClimbMotorMaximumSpeed)
      speed = -MotorConstants.kClimbMotorMaximumSpeed;
    m_ClimbMotor.set(speed);
    SmartDashboard.putNumber("Climb speed", speed);
  }

  public void stopClimbMotor() {
    m_ClimbMotor.stopMotor();
    SmartDashboard.putNumber("Climb speed", 0);
  }

  public RelativeEncoder getRelativeEncoder() {
    return m_RelativeEncoder;
  }

  public double getRelativeEncoderPosition() {
    return m_RelativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb position", getRelativeEncoderPosition()); //does not work
  }

  @Override
  public void simulationPeriodic() {}
}
