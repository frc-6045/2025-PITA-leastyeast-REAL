package frc.robot.commands.AutoScoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoScoreConstants.Side;
import frc.robot.Constants.PositionConstants.Setpoints;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private Setpoints m_setpoint;
    private Side m_side;
    private Translation2d m_offset;
    private Pose2d m_closestTag;
    private SwerveSubsystem m_swerveSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    private ElevatorSubsystem m_ElevatorSubsystem; 

    public AutoAlign(Setpoints setpoint, Supplier<Side> side, Supplier<Translation2d> offset, Supplier<Pose2d> closestTag, SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_setpoint = setpoint;
        m_side = side.get();
        m_offset = offset.get();
        m_closestTag = closestTag.get();
        m_swerveSubsystem = swerveSubsystem;
        m_ArmSubsystem = armSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;

        addRequirements(m_swerveSubsystem, m_ArmSubsystem, m_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d firstPoseDriveTo, secondPoseDriveTo;

        // straight behind the april tag by quite a bit
        firstPoseDriveTo = AutoScoreCommands.applyOffsetToPose(m_closestTag, new Translation2d(-1,0));

        // left or right side of reef
        double sideOffset = (m_side == Side.RIGHT) ? -0.2 : 0.2;
        secondPoseDriveTo = AutoScoreCommands.applyOffsetToPose(m_closestTag, new Translation2d(0.6, sideOffset));

        // intake offset
        secondPoseDriveTo = AutoScoreCommands.applyOffsetToPose(secondPoseDriveTo, m_offset);

        System.out.println("align to nearest reef face, side is " + m_side +
        "\napriltag pose is " + m_closestTag.getX() + " " + m_closestTag.getY() + " " + m_closestTag.getRotation() +
        "\ncoral offset is " + m_offset.getX() + " " + m_offset.getY());

        new SequentialCommandGroup(
            m_swerveSubsystem.driveToPose(firstPoseDriveTo),
            new ParallelDeadlineGroup(
                m_swerveSubsystem.driveToPoseSlowMode(secondPoseDriveTo),
                new PIDArmAndElevator(m_ArmSubsystem, m_ElevatorSubsystem, m_setpoint)
            )
        ).schedule();
    }
}
