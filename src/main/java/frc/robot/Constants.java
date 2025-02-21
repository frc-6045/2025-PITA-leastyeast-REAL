// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MotorConstants {
    // arm constants
    public static final int kSparkFlexArmMotorCANID = 9;

    public static final int kSparkFlexArmMotorCurrentLimit = 40;
    public static final double kSparkFlexArmMotorSpeed = .2;
    public static final double kSparkFlexArmMotorMaxSpeed = 0.420;

    // elevator constants
    public static final int kSparkFlexElevatorMotor1CANID = 10;
    public static final int kSparkFlexElevatorMotor2CANID = 11;

    public static final int kSparkFlexElevatorMotorsCurrentLimit = 40;
    public static final double kSparkFlexElevatorMotorsSpeed = .65;
    public static final double kSparkFlexElevatorMotorsMaxSpeed = 0.95;

    // intake constants
    public static final int kIntakeMotor1CANID = 12;
    public static final int kIntakeMotor2CANID = 13;

    public static final int kIntakeMotorsCurrentLimit = 40;
    public static final double kIntakeMotorsSpeed = .5;
    public static final double kIntakeMotorsMaxSpeed = .6;

    //algae-removing constants
    public static final int kAlgaeRemovingMotorCANID = 14;

    public static final int kAlgaeRemovingMotorCurrentLimit = 40;
    public static final double kAlgaeRemovingMotorSpeed = 0.5;
    public static final double kAlgaeRemovingMotorMaxSpeed = 0.6;

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

    // A- Human player/coral intake setpoint
    public static final double kHumanArmPosition = 0.4262903034687042;
    public static final double kHumanElevatorPosition = 59;

    // Y- Home setpoint
    public static final double kHomeArmPosition = 0.32972583174705505;
    public static final double kHomeElevatorPosition = 14;

    // B- L1
    public static final double kL1ArmPosition = 0.46141165494918823;
    public static final double kL1ElevatorPosition = 0;

    // L2
    public static final double kL2ArmPosition = 0.277045;
    public static final double kL2ElevatorPosition = 22.1052;

    // left stick (top left paddle)- L3
    public static final double kL3ArmPosition = 0.9638405442237854;
    public static final double kL3ElevatorPosition = 0;

    // right stick (top right paddle)- L4
    public static final double kL4ArmPosition = 0.9521994590759277;
    public static final double kL4ElevatorPosition = 100;

    // arm flick goes to initialposition+kArmFlickDistance1 then to initialposition+kArmFlickDistance2
    public static final double kArmFlickDistance1=0.05;
    public static final double kArmFlickDistance2=-0.01;
  }

  public static class SwerveConstants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(14.34);
  }
  
}

// maek controler wurk plez