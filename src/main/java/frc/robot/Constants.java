// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


public final class Constants {
  /**  <p>MotorSpeed is the open loop speed.
   *  <p>MotorMaxSpeed is a hard limit on speed.
   */
  public static class MotorConstants {

    // arm constants
    public static final int kSparkFlexArmMotorCANID = 9;

    public static final int kSparkFlexArmMotorCurrentLimit = 40;
    public static final double kSparkFlexArmMotorSpeed = .2;
    public static final double kSparkFlexArmMotorMaxSpeed = 0.55;

    // elevator constants
    public static final int kSparkFlexElevatorMotor1CANID = 10;
    public static final int kSparkFlexElevatorMotor2CANID = 11;

    public static final int kSparkFlexElevatorMotorsCurrentLimit = 40;
    public static final double kSparkFlexElevatorMotorsSpeed = .4;
    public static final double kSparkFlexElevatorMotorsMaxSpeed = 0.7;

    // intake constants
    public static final int kIntakeMotor1CANID = 12;
    public static final int kIntakeMotor2CANID = 13;

    public static final int kIntakeMotorsCurrentLimit = 40;
    public static final double kIntakeMotorsSpeed = .9;
    public static final double kIntakeMotorsMaxSpeed = .91;

    //algae-removing constants
    public static final int kAlgaeRemovingMotorCANID = 14;

    public static final int kAlgaeRemovingMotorCurrentLimit = 40;
    public static final double kAlgaeRemovingMotorSpeed = 0.99;
    public static final double kAlgaeRemovingMotorMaxSpeed = 1;

    //climb constants
    public static final int kClimbMotorCANID = 15;

    public static final int kClimbMotorCurrentLimit = 50;
    public static final double kClimbMotorSpeed = 0.5;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 0;
    public static final int kGodControllerPort = 2;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PositionConstants {
    public static final double kOffset = -.17;
    public static final double kSketchyOffset = -0.4342;

    // Human player/coral intake setpoint - A
    public static final double kHumanArmPosition = 0.7170278 + kOffset;
    public static final double kHumanElevatorPosition = -39.62 + kOffset;

    // Human player with gap
    public static final double kHumanGapArmPosition = 0.68967 + kOffset;
    public static final double kHumanGapElevatorPosition = -29.3663 + kOffset;

    // Home setpoint - Y
    public static final double kHomeArmPosition = 0.9063 + kOffset;
    public static final double kHomeElevatorPosition = 0 + kOffset;

    // L1 - B
    public static final double kL1ArmPosition = 0.937577 + kOffset;
    public static final double kL1ElevatorPosition = -2.29429 + kOffset;

    // L2
    public static final double kL2ArmPosition = 0.8916632533 + kOffset;
    public static final double kL2ElevatorPosition = -33.58844757 + kOffset;

    // L3 - left stick (top left paddle)
    public static final double kL3ArmPosition = 0.22141972 + kOffset;
    public static final double kL3ElevatorPosition = -1.62234926 + kOffset;

    // L3 Gap
    public static final double kL3GapArmPosition = 0.1960784345 + kOffset;
    public static final double kL3GapElevatorPosition = -16.60658489 + kOffset;

    // L4 - right stick (top right paddle)
    public static final double kL4ArmPosition = 0.18726785 + kOffset;
    public static final double kL4ElevatorPosition = -77.241875 + kOffset;

    // L4 Gap
    public static final double kL4GapArmPosition = 0.203837424 + kOffset;
    public static final double kL4GapElevatorPosition = -76.78836822 + kOffset;

    // algae high
    public static final double kHighAlgaeArmPosition = 0.27118 + kOffset;
    public static final double kHighAlgaeElevatorPosition = -20.3655 + kOffset;
    
    // algae low
    public static final double kLowAlgaeArmPosition = 0.212592096 + kOffset;
    public static final double kLowAlgaeElevatorPosition = -0.3056640625 + kOffset;

    // arm flick goes to initialposition+kArmFlickDistance1 then to initialposition+kArmFlickDistance2
    public static final double kArmFlickDistance1=0.05;
    public static final double kArmFlickDistance2=-0.01;

    // algae shoot setpoints
    public static final double kAlgaeShootArm = 0;
    public static final double kAlgaeShootElev = 0;

    //nonospaces that make turnbuckle vewy vewy sad :(
    public static final double kArmLimit1=.172-.01;
    public static final double kArmLimit2=0.4342+.01;
    public static final double kMiddleOfArmLimit = (kArmLimit1+kArmLimit2)/2;
  }

  public static class SwerveConstants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(25);
  }
  
}

// maek controler wurk plez