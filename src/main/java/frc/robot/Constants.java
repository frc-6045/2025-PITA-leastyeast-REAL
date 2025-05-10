package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  public static final String LIMELIGHT = "limelight-sabre";

  //  MotorSpeed is the open loop speed.
  //  MotorMaxSpeed is a hard limit on speed.
  public static class MotorConstants {

    // CAN IDs
    public static final int kArmMotorCANID = 9;
    public static final int kElevatorMotor1CANID = 10;
    public static final int kElevatorMotor2CANID = 11;
    public static final int kIntakeMotorCANID = 12;
    public static final int kClimbWristMotorCANID = 13;
    public static final int kClimbMotorCANID = 15;

    // arm constants
    public static final int kArmMotorCurrentLimit = 40;
    public static final double kArmMotorSpeed = .2;
    public static final double kArmMotorSetpointMaxSpeed = 0.85;
    public static final double kArmMotorBargeMaxSpeed = 0.4;
    public static final double kArmMotorMaxSpeed = 0.85;

    // elevator constants
    public static final int kElevatorMotorsCurrentLimit = 40;
    public static final double kElevatorMotorsSpeed = .4;
    public static final double kElevatorSetpointMaxSpeed = 0.7;
    public static final double kElevatorMotorsMaxSpeed = 0.7;

    // intake constants
    public static final int kIntakeMotorCurrentLimit = 40;
    public static final double kOperatorIntakeMotorSpeed = 1;
    public static final double kIntakeMotorSpeed = .99999;
    public static final double kIntakeMotorMaxSpeed = 1;

    //climb constants
    public static final int kClimbMotorCurrentLimit = 50;
    public static final double kClimbMotorSpeed = 0.9;
    public static final double kClimbMotorMaximumSpeed = 0.91;

    public static final int kWristCurrentLimit = 50;
    public static final double kWristMotorSpeed = 1;
    public static final double kWristMotorMaxSpeed = 1;
  }

  public static class ControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kTestControllerPort = 2;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PositionConstants {
    // kOffset is the encoder offset, so it is easy to refactor all setpoints when the encoder is moved.
    // kSketchyOffset makes PID go the right way. It essentially gives a convenient spot (that being a bit behind the intake setpoint) the value of 0.
    // It is likely possible to combine these offsets, but this works for now!
    public static final double kOffset = 0; // likely will never be used
    public static final double kSketchyOffset = 0.33;

    // SETPOINTS

    // Human player/coral intake setpoint - A
    public static final double kHumanArmPosition = 0.9378 + kOffset;
    public static final double kHumanElevatorPosition = -40.38;

    // Human player with gap
    public static final double kHumanGapArmPosition = 0.8896;
    public static final double kHumanGapElevatorPosition = 0;

    // Home setpoint - Y
    public static final double kHomeArmPosition = 0.8896 + kOffset;
    public static final double kHomeElevatorPosition = 0;

    // L1 - B
    public static final double kL1ArmPosition = 0.716 + kOffset;
    public static final double kL1ElevatorPosition = 0;

    // L2
    public static final double kL2ArmPosition = 0.7543 + kOffset;
    public static final double kL2ElevatorPosition = -26.156;

    // L3 - left stick (top left paddle)
    public static final double kL3ArmPosition = 0.4526 + kOffset;
    public static final double kL3ElevatorPosition = 0;

    // L3 Gap
    public static final double kL3GapArmPosition = 0.8896;
    public static final double kL3GapElevatorPosition = -16.60658489;

    // L4 - right stick (top right paddle)
    public static final double kL4ArmPosition = 0.46166+ kOffset;
    public static final double kL4ElevatorPosition = -74.67;

    // L4 Gap
    public static final double kL4GapArmPosition = 0.8896 + kOffset;
    public static final double kL4GapElevatorPosition = -76.78836822;

    // algae high
    public static final double kHighAlgaeArmPosition = 0.6402 + kOffset;
    public static final double kHighAlgaeElevatorPosition = -50.00;
    
    // algae low
    public static final double kLowAlgaeArmPosition = 0.6567 + kOffset;
    public static final double kLowAlgaeElevatorPosition = -8.89;
    
    // barge
    public static final double kBargeArm = 0.331647 + kOffset;
    public static final double kBargeElev = -77.2;

    // lollipop
    public static final double kLollipopArm = .75626;
    public static final double kLollipopElev = 0;

    //nonospaces that make turnbuckle vewy vewy sad :(
    public static final double kArmLimit1=0.884;
    public static final double kArmLimit2=0.999;
    public static final double kMiddleOfArmLimit = (kArmLimit1+kArmLimit2)/2;

    public static enum Setpoints {
      INTAKE,
      HOME,
      L1,
      L2,
      L3,
      L4,
      ALGAE_HIGH,
      ALGAE_LOW,
      BARGE,
      LOLLIPOP
    }
    // LimeLight Constants
    public static final double Y_REEF_ALIGNMENT_P = 0.1;
    public static final double ROT_REEF_ALIGNMENT_P = 1.5;
  }


  public static class SwerveConstants {
    public static final double ROBOT_MASS = (149) * 0.453592; // lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(25);
  }

  public static class AutoScoreConstants {
    public static final Pose2d REEF_FACE_ONE = new Pose2d(3.818, 4.014, new Rotation2d());  //AB 
    public static final Pose2d REEF_FACE_TWO = new Pose2d(4.153, 3.416, new Rotation2d());  //CD
    public static final Pose2d REEF_FACE_THREE = new Pose2d(4.842, 3.422, new Rotation2d());//EF
    public static final Pose2d REEF_FACE_FOUR = new Pose2d(5.177, 4.022, new Rotation2d()); //GH
    public static final Pose2d REEF_FACE_FIVE = new Pose2d(4.840, 4.596, new Rotation2d()); //IJ
    public static final Pose2d REEF_FACE_SIX = new Pose2d(4.153, 4.014, new Rotation2d());  //KL
    public static final Pose2d[] REEF_FACE_ARRAY = new Pose2d[] {
      REEF_FACE_ONE, 
      REEF_FACE_TWO, 
      REEF_FACE_THREE, 
      REEF_FACE_FOUR, 
      REEF_FACE_FIVE, 
      REEF_FACE_SIX};
    public static enum Side {
      LEFT,
      RIGHT
    }
    public static final Pose2d PoleA = new Pose2d(3.153, 4.175, Rotation2d.fromDegrees(180));
    public static final Pose2d PoleB = new Pose2d(3.189, 3.827, Rotation2d.fromDegrees(180));
    public static final Pose2d PoleC = new Pose2d(3.704, 2.952, Rotation2d.fromDegrees(-120));
    public static final Pose2d PoleD = new Pose2d(3.956, 2.820, Rotation2d.fromDegrees(-120));
    public static final Pose2d PoleE = new Pose2d(4.975, 2.784, Rotation2d.fromDegrees(-61));
    public static final Pose2d PoleF = new Pose2d(5.275, 2.940, Rotation2d.fromDegrees(-61));
    public static final Pose2d PoleG = new Pose2d(5.790, 3.851, Rotation2d.fromDegrees(0));
    public static final Pose2d PoleH = new Pose2d(5.802, 4.187, Rotation2d.fromDegrees(0));
    public static final Pose2d PoleI = new Pose2d(5.335, 5.050, Rotation2d.fromDegrees(59));
    public static final Pose2d PoleJ = new Pose2d(4.999, 5.242, Rotation2d.fromDegrees(59));
    public static final Pose2d PoleK = new Pose2d(3.956, 5.206, Rotation2d.fromDegrees(123));
    public static final Pose2d PoleL = new Pose2d(3.680, 5.074, Rotation2d.fromDegrees(123));
    
    // distance sensor values
    public static final double coralLocation0 = 0.11; // no coral
    public static final double coralLocation1 = 0.080;
    public static final double coralLocation2 = 0.062;
    public static final double coralLocation3 = 0.046;
    public static final double coralLocation4 = 0.033;
    
    // offsets
    public static final Translation2d autoScoreCoralOffset1 = new Translation2d();
    public static final Translation2d autoScoreCoralOffset2 = new Translation2d();
    public static final Translation2d autoScoreCoralOffset3 = new Translation2d();
    public static final Translation2d autoScoreCoralOffset4 = new Translation2d();
  }
}
