package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.PIDArmAndElevator;
import frc.robot.commands.ElevatorCommands.ElevatorDown;
import frc.robot.commands.IntakeCommands.IntakeClosedLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autos {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private SendableChooser<Command> autoChooser;

    public Autos(SwerveSubsystem drive, IntakeSubsystem intake, ElevatorSubsystem elev, ArmSubsystem arm) {
        m_IntakeSubsystem = intake;
        m_ElevatorSubsystem = elev;
        m_ArmSubsystem = arm;

        // Named Commands
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
        NamedCommands.registerCommand("coralSpit", new IntakeClosedLoop(m_IntakeSubsystem, 1, false));
        NamedCommands.registerCommand("coralIntake", new IntakeClosedLoop(m_IntakeSubsystem, 0.5, true));
        NamedCommands.registerCommand("elevatorDown", new ElevatorDown(m_ElevatorSubsystem, 1));
        NamedCommands.registerCommand("coralL1", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL1ArmPosition, m_ElevatorSubsystem, PositionConstants.kL1ElevatorPosition));
        NamedCommands.registerCommand("coralL2", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL2ArmPosition, m_ElevatorSubsystem, PositionConstants.kL2ElevatorPosition));
        NamedCommands.registerCommand("coralL3", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL3ArmPosition, m_ElevatorSubsystem, PositionConstants.kL3ElevatorPosition));
        NamedCommands.registerCommand("coralL4", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kL4ArmPosition, m_ElevatorSubsystem, PositionConstants.kL4ElevatorPosition));
        NamedCommands.registerCommand("coralIntakeSetpoint", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHumanArmPosition, m_ElevatorSubsystem, PositionConstants.kHumanElevatorPosition));
        NamedCommands.registerCommand("homePosition", new PIDArmAndElevator(m_ArmSubsystem, PositionConstants.kHomeArmPosition, m_ElevatorSubsystem, PositionConstants.kHomeElevatorPosition));

        // Autos
        autoChooser = new SendableChooser<Command>();
        //autoChooser.addOption("Do Nothing", new InstantCommand(() -> {System.out.println("hi");}));
        autoChooser.addOption("1PieceIPole", AutoBuilder.buildAuto("1PieceIPole"));
        autoChooser.addOption("2PieceIKPoles", AutoBuilder.buildAuto("2PieceIKPoles"));
        autoChooser.addOption("3PieceIKJPoles", AutoBuilder.buildAuto("3PieceIKJPoles"));
        autoChooser.addOption("1PieceHPole", AutoBuilder.buildAuto("1PieceHPole"));
        autoChooser.addOption("2PieceHGPoles", AutoBuilder.buildAuto("2PieceHGPoles"));
        autoChooser.addOption("1PieceL1Center", AutoBuilder.buildAuto("1PieceL1Center"));
        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
