package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    //private final SwerveSubsystem m_DriveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;
        //m_DriveSubsystem = drive;

        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("coralSpit", new IntakeClosedLoop(m_IntakeSubsystem, 1, false));
        NamedCommands.registerCommand("coralIntake", new IntakeClosedLoop(m_IntakeSubsystem, 1, true));
        NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
        NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
        NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
        NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
        NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));

        //incorporate pathplanner into code
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("Do Nothing", new InstantCommand(() -> {System.out.println("hi");}));
        autoChooser.addOption("SetpointThenIntake", AutoBuilder.buildAuto("SetpointThenIntake"));
        autoChooser.addOption("3piece", AutoBuilder.buildAuto("3PieceJKLTesting"));
        autoChooser.addOption("BackStart2Coral", AutoBuilder.buildAuto("BackStart2Coral"));
        autoChooser.addOption("3PieceJKLPolesWithCommands", AutoBuilder.buildAuto("3PieceJKLPoles"));
        autoChooser.addOption("3PieceJKLPolesDriveOnly", AutoBuilder.buildAuto("3PieceJKLPolesOld"));
        autoChooser.addOption("DriveForwordAndIntakeTest", AutoBuilder.buildAuto("DriveForwordAndIntakeTest"));
        autoChooser.addOption("MiddleStart1Coral", AutoBuilder.buildAuto("MiddleStart1Coral"));
        SmartDashboard.putData("autos", autoChooser);
        autoChooser.addOption("IJKLPoles1PieceDrive", AutoBuilder.buildAuto("1PieceIPoleDrive"));
        autoChooser.addOption("IJKLPoles2PieceDrive", AutoBuilder.buildAuto("2PieceIJPolesDrive"));
        autoChooser.addOption("IJKLPoles3PieceDrive", AutoBuilder.buildAuto("3PieceIJKPolesDrive"));
        autoChooser.addOption("IJKLPoles4PieceDrive", AutoBuilder.buildAuto("4PieceIJKLPolesDrive"));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
