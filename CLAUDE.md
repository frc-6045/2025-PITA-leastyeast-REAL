# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) 2025 robot codebase for Team 6045, written in Java using the WPILib command-based framework. The robot is designed for the 2025 Reefscape game and features a swerve drive system with multiple game-piece handling mechanisms.

## Build System and Commands

This project uses Gradle with the GradleRIO plugin for FRC development.

### Essential Commands

```bash
# Build the robot code
./gradlew build

# Deploy code to the robot (RoboRIO)
./gradlew deploy

# Run tests
./gradlew test

# Clean build artifacts
./gradlew clean

# Build without running tests
./gradlew assemble
```

### Robot Simulation

```bash
# Run robot simulation with GUI
./gradlew simulateJava
```

The simulation includes DriverStation and GUI by default (configured in build.gradle).

## Project Configuration

- **Team Number**: 6045 (configured in `.wpilib/wpilib_preferences.json`)
- **Java Version**: 17
- **WPILib Year**: 2025
- **Robot Main Class**: `frc.robot.Main`

## Code Architecture

### Command-Based Framework Structure

The codebase follows WPILib's command-based architecture:

1. **Robot.java** - Entry point, extends `TimedRobot`, calls `CommandScheduler.getInstance().run()` every 20ms
2. **RobotContainer.java** - Central configuration point that:
   - Instantiates all subsystems
   - Creates controller bindings via `Bindings.InitBindings()`
   - Configures drivetrain defaults
   - Returns autonomous commands
3. **Bindings.java** - Separates controller button mappings from RobotContainer for organization
4. **Constants.java** - Nested static classes for all robot constants (motor IDs, speeds, positions, etc.)
5. **Autos.java** - Manages autonomous routines using PathPlanner, registers named commands, and provides auto chooser

### Subsystems

Located in `src/main/java/frc/robot/subsystems/`:

- **SwerveSubsystem** - Swerve drive using YAGSL (Yet Another Generic Swerve Library)
  - Configuration loaded from JSON files in `src/main/deploy/swerve/neo/`
  - Integrated with PathPlanner for autonomous path following
  - Contains auto-scoring helper methods (`getNearestPole`, `driveToFirstAutoScorePose`, etc.)
  - Uses AprilTag field layout for vision alignment
- **ArmSubsystem** - Arm mechanism for game piece manipulation
- **ElevatorSubsystem** - Elevator with limit switches for scoring at different heights
- **IntakeSubsystem** - Coral/algae game piece intake with detection sensors
- **ClimbSubsystem** - End-game climb mechanism
- **ClimbWristSubsystem** - Wrist for climb positioning
- **LedSubsystem** - LED indicators for robot state

### Commands Organization

Commands are organized by subsystem in subdirectories:
- `commands/ArmCommands/` - Arm control (open loop, PID, hold)
- `commands/ElevatorCommands/` - Elevator control
- `commands/IntakeCommands/` - Intake operations (intake, outtake, conditional control)
- `commands/swerve/drivebase/` - Swerve drive commands
- `commands/AutoScoring/` - Vision-assisted scoring commands using Limelight

The **PIDArmAndElevator** command (root commands directory) coordinates arm and elevator movements to predefined setpoints.

### Constants Structure

Constants.java uses nested classes for organization:
- **MotorConstants** - CAN IDs, current limits, speed limits for all motors
- **ControllerConstants** - Controller ports and joystick deadbands
- **PositionConstants** - Arm/elevator setpoints for scoring positions (L1-L4, algae, barge, etc.)
  - Uses enum `Setpoints` for type-safe setpoint references
  - Includes `kSketchyOffset` for PID tuning convenience
- **SwerveConstants** - Robot mass, loop time, max speed
- **AutoScoreConstants** - Reef face poses, pole positions, coral sensor values

### PathPlanner Integration

- Autonomous paths are defined in `src/main/deploy/pathplanner/`
- **Named Commands** registered in `Autos.java` for use in PathPlanner GUI:
  - Scoring commands: `coralL1`, `coralL2`, `coralL3`, `coralL4`, `algaeHigh`, `algaeLow`, `barge`
  - Intake/outtake: `coralIntake`, `coralSpinNormal`, `coralSpinOther`, `algaeInOne/Two/Three`
  - Position commands: `homePosition`, `coralIntakeSetpoint`, `LOLLIPOP`
- AutoBuilder configured in `SwerveSubsystem.setupPathPlanner()`
- Uses RobotConfig from PathPlanner GUI settings

### Game-Specific Logic

**2025 Reefscape Game Elements**:
- **Coral** - Primary game piece, detected by intake sensors
- **Algae** - Secondary game piece
- **Reef** - Scoring structure with 6 faces (A-L poles) and 4 levels (L1-L4)

**Scoring Positions** (in PositionConstants):
- L1-L4: Four scoring levels on the reef
- Human: Human player station intake position
- Algae High/Low: Algae processing positions
- Barge: Special scoring technique using coordinated arm/elevator/intake
- Home: Default safe position
- Lollipop: Named setpoint for specific game strategy

**Auto-Scoring System**:
- Uses Limelight camera (name: "limelight-sabre") for vision alignment
- `AlignToReefTagRelative` command for AprilTag-based alignment
- Distance sensor values for coral positioning (coralLocation0-4 in AutoScoreConstants)
- Robot can determine nearest reef face and appropriate pole based on current pose

## Vendor Dependencies

Key libraries installed (vendordeps/):
- **YAGSL** (yagsl-2025.7.2.json) - Swerve drive library
- **PathPlannerLib** (PathplannerLib-2025.2.6.json) - Path planning and following
- **REVLib** - REV Robotics motor controllers (Spark Max with NEO motors)
- **Phoenix6** - CTRE motor controllers and sensors
- **Phoenix5** (legacy) - Older CTRE devices
- Additional: ReduxLib, ThriftyLib, PlayingWithFusion, Studica, maple-sim

## Testing

- JUnit 5 configured for unit tests
- Test configuration in `.vscode/settings.json` sets up native library paths
- Auto-detection enabled for JUnit extensions
- Working directory: `${workspaceFolder}/build/jni/release`

## Development Workflow

1. Make code changes in `src/main/java/frc/robot/`
2. Build locally with `./gradlew build` to check for compile errors
3. Test in simulation with `./gradlew simulateJava` if applicable
4. Deploy to robot with `./gradlew deploy` when connected to robot network
5. For autonomous development, use PathPlanner GUI to create/edit paths, ensure named commands are registered in Autos.java

## Important Notes

- **Encoder Zeroing**: Elevator has a bottom limit switch that zeros the encoder (see Bindings.java line 104)
- **Alliance Color**: Many commands (swerve, autos) are alliance-aware and will mirror for red alliance
- **Coordinate System**: Field-relative control is alliance-relative (blue origin on blue side, paths mirror for red)
- **Swerve Configuration**: Module configs are in deploy directory, loaded at runtime from JSON
- **Default Commands**: Arm uses `HoldArm`, Elevator uses `HoldElevator` to maintain position when idle
- **Controller Bindings**: Three Xbox controllers supported (driver, operator, test/vision)
