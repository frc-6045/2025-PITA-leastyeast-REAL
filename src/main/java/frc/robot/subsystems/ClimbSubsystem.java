//make subsystem

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

//import edu.wpi.first.math.controller.ClimbFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PositionConstants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkFlex m_ClimbMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  SparkFlexConfig config = new SparkFlexConfig();
  //private final ClimbFeedforward m_ClimbFeedforward = new ClimbFeedforward(0, 0, 0);
  PIDController m_ClimbPIDController = new PIDController(9, 0, 0);

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    m_ClimbMotor = new SparkFlex(MotorConstants.kClimbMotorCANID, MotorType.kBrushless);
    m_AbsoluteEncoder = m_ClimbMotor.getAbsoluteEncoder();
    //m_ClimbPIDController.enableContinuousInput(0, 1);
    m_ClimbPIDController.setTolerance(0.01);

    updateMotorSettings(m_ClimbMotor);
    m_ClimbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

   public void updateMotorSettings(SparkFlex motor) {
    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kIntakeMotorsCurrentLimit);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
  }

  public void goToSetpoint(double setpoint) {
    SmartDashboard.putNumber("Climb setpoint", setpoint+PositionConstants.kSketchyOffset);
    SmartDashboard.putNumber("Climb difference", setpoint-getAbsoluteEncoderPosition());
    double speed = m_ClimbPIDController.calculate((getAbsoluteEncoderPosition()+14+PositionConstants.kSketchyOffset)%1, (setpoint+14+PositionConstants.kSketchyOffset)%1);
    //speed = (speed>0) ? speed + feedforward : speed-feedforward;
    setSpeed(speed);
    //System.out.println("PIDClimb output (speed): " + speed + "\nset point: " + m_ClimbPIDController.getSetpoint() + "\ncurrent position: " + getAbsoluteEncoderPosition());
  }

  public boolean atSetpoint() {
    return m_ClimbPIDController.atSetpoint();
  }

  public void setSpeed(double speed) {
    if (speed>MotorConstants.kClimbMotorMaximumSpeed)
      speed = MotorConstants.kClimbMotorMaximumSpeed;
    if (speed<-MotorConstants.kClimbMotorMaximumSpeed)
      speed = -MotorConstants.kClimbMotorMaximumSpeed;
    
    // prevent turnbuckle from being run over
    


    m_ClimbMotor.set(speed);
    SmartDashboard.putNumber("Climb speed", speed);
  }

  public void stopClimbMotor() {
    m_ClimbMotor.stopMotor();
    SmartDashboard.putNumber("Climb speed", 0);
  }

  public AbsoluteEncoder getAbsoluteEncoder() {
    return m_AbsoluteEncoder;
  }

  public double getAbsoluteEncoderPosition() {
    return m_AbsoluteEncoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climb position", getAbsoluteEncoderPosition()+PositionConstants.kSketchyOffset);
    SmartDashboard.putNumber("raw Climb position", getAbsoluteEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
