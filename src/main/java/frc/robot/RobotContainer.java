

  package frc.robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  
  public final SwerveSubsystem m_DriveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  // define controllers
  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorControllerPort);
  private final CommandXboxController m_driverController = 
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  private final CommandXboxController m_testVisionController =
      new CommandXboxController(ControllerConstants.kTestControllerPort);   

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Bindings.InitBindings(m_operatorController, m_driverController, m_testVisionController, m_DriveSubsystem, m_IntakeSubsystem);
    Bindings.configureDrivetrain(m_DriveSubsystem, m_driverController);
    DriverStation.silenceJoystickConnectionWarning(true);
  }
  
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}