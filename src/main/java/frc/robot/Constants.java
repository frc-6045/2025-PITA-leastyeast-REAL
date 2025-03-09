// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  /**  MotorSpeed is the open loop speed.
   *  MotorMaxSpeed is a hard limit on speed.
   */
  public static class MotorConstants {

    // CAN IDs
    public static final int kArmMotorCANID = 9;
    public static final int kElevatorMotor1CANID = 10;
    public static final int kElevatorMotor2CANID = 11;
    public static final int kIntakeMotorCANID = 12;
    public static final int kClimbMotorCANID = 15;

    // arm constants
    public static final int kArmMotorCurrentLimit = 40;
    public static final double kArmMotorSpeed = .2;
    public static final double kArmMotorSetpointMaxSpeed = 0.3;
    public static final double kArmMotorBargeMaxSpeed = 0.4;
    public static final double kArmMotorMaxSpeed = 0.55;

    // elevator constants
    public static final int kElevatorMotorsCurrentLimit = 40;
    public static final double kElevatorMotorsSpeed = .4;
    public static final double kElevatorSetpointMaxSpeed = 0.7;
    public static final double kElevatorMotorsMaxSpeed = 0.7;

    // intake constants
    public static final int kIntakeMotorCurrentLimit = 40;
    public static final double kIntakeMotorSpeed = .9;
    public static final double kIntakeMotorMaxSpeed = .91;

    //climb constants
    public static final int kClimbMotorCurrentLimit = 50;
    public static final double kClimbMotorSpeed = 0.9;
    public static final double kClimbMotorMaximumSpeed = 0.91;
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kGodControllerPort = 2;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PositionConstants {
    // LimeLight Constants
    public static final double X_REEF_ALIGNMENT_P = 1.0;
    public static final double Y_REEF_ALIGNMENT_P = 1.0;
    public static final double ROT_REEF_ALIGNMENT_P = 0.06;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;

    public static final double X_SETPOINT_REEF_ALIGNMENT = 0.0;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;

    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.19;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.005;
    
    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;

    // kOffset is the encoder offset, so it is easy to refactor all setpoints when the encoder is moved.
    // kSketchyOffset makes PID go the right way. It essentially gives a convenient spot (that being a bit behind the intake setpoint) the value of 0.
    // It is likely possible to combine these offsets, but this works for now!
    public static final double kOffset = 0; // likely will never be used
    public static final double kSketchyOffset = 0.33;

    // Human player/coral intake setpoint - A
    public static final double kHumanArmPosition = 0.9561 + kOffset;
    public static final double kHumanElevatorPosition = -40.229;

    // Human player with gap
    public static final double kHumanGapArmPosition = 0.8896; //temp
    public static final double kHumanGapElevatorPosition = 0;

    // Home setpoint - Y
    public static final double kHomeArmPosition = 0.8896 + kOffset;;
    public static final double kHomeElevatorPosition = 0;

    // L1 - B
    public static final double kL1ArmPosition = 0.716 + kOffset;;
    public static final double kL1ElevatorPosition = 0;

    // L2
    public static final double kL2ArmPosition = 0.7473 + kOffset;
    public static final double kL2ElevatorPosition = -20.9609;

    // L3 - left stick (top left paddle)
    public static final double kL3ArmPosition = 0.4526 + kOffset;;
    public static final double kL3ElevatorPosition = 0;

    // L3 Gap
    public static final double kL3GapArmPosition = 0.8896; //temp
    public static final double kL3GapElevatorPosition = -16.60658489;

    // L4 - right stick (top right paddle)
    public static final double kL4ArmPosition = 0.4526 + kOffset;;
    public static final double kL4ElevatorPosition = -75.17;

    // L4 Gap
    public static final double kL4GapArmPosition = 0.8896 + kOffset;; //temp
    public static final double kL4GapElevatorPosition = -76.78836822;

    // algae high
    public static final double kHighAlgaeArmPosition = 0.6273 + kOffset;;
    public static final double kHighAlgaeElevatorPosition = -36.172;
    
    // algae low
    public static final double kLowAlgaeArmPosition = 0.6486 + kOffset;;
    public static final double kLowAlgaeElevatorPosition = -2;
    
    // barge
    public static final double kBargeArm = 0.331647 + kOffset;;
    public static final double kBargeElev = 0; //-77.2

    // arm flick goes to initialposition+kArmFlickDistance1 then to initialposition+kArmFlickDistance2
    public static final double kArmFlickDistance1=0.05;
    public static final double kArmFlickDistance2=-0.01;

    //nonospaces that make turnbuckle vewy vewy sad :(
    public static final double kArmLimit1=0.884; //0.06
    public static final double kArmLimit2=0.98; // 0.26
    public static final double kMiddleOfArmLimit = (kArmLimit1+kArmLimit2)/2;

    //just having fun
    public static enum Setpoints {
      INTAKE,
      HOME,
      L1,
      L2,
      L3,
      L4,
      ALGAE_HIGH,
      ALGAE_LOW,
      BARGE
    }
  }

  public static class SwerveConstants {
    public static final double ROBOT_MASS = (109) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(25);
  }
  
}

// maek controler wurk plez