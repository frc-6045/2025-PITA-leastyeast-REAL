package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
import frc.robot.commands.SwerveCommands.AlignToReefTagRelative;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final SwerveSubsystem m_SwerveSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_SwerveSubsystem = drive;
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;

        // Named Commands
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("coralSpit", new IntakeClosedLoop(m_IntakeSubsystem, 1, false));
        NamedCommands.registerCommand("coralIntake", new IntakeClosedLoop(m_IntakeSubsystem, 0.5, true));
        NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition).asProxy());
        NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition).asProxy());
        NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition).asProxy());
        NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition).asProxy());
        NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition).asProxy());
        NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition).asProxy());
        NamedCommands.registerCommand("alignToReefLeft", new AlignToReefTagRelative(false, m_SwerveSubsystem));
        NamedCommands.registerCommand("alignToReefRight", new AlignToReefTagRelative(true, m_SwerveSubsystem));

        // NamedCommands.registerCommand("scoreCoralL1",
        //     new SequentialCommandGroup(
        //         new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition).asProxy(),
        //         new AlignToReefTagRelative(m_SwerveSubsystem),
        //         new IntakeClosedLoop(m_IntakeSubsystem, 1, false)
        //     )
        // );

        // Autos
        autoChooser = new SendableChooser<Command>();
        //autoChooser.addOption("Do Nothing", new InstantCommand(() -> {System.out.println("hi");}));
        autoChooser.addOption("1PieceIPole", AutoBuilder.buildAuto("1PieceIPole"));
        autoChooser.addOption("2PieceIKPoles", AutoBuilder.buildAuto("2PieceIKPoles"));
        autoChooser.addOption("3PieceIKJPoles", AutoBuilder.buildAuto("3PieceIKJPoles"));
        autoChooser.addOption("1PieceHPole", AutoBuilder.buildAuto("1PieceHPole"));
        autoChooser.addOption("2PieceHGPoles", AutoBuilder.buildAuto("2PieceHGPoles"));
        autoChooser.addOption("1PieceL1Center", AutoBuilder.buildAuto("1PieceL1Center"));
        autoChooser.addOption("Test Auto", AutoBuilder.buildAuto("New Auto"));  
        autoChooser.addOption("3PieceIKJPolesLimelight", AutoBuilder.buildAuto("3PieceIKJPolesLimelight"));
        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
