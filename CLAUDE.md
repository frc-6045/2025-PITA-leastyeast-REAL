# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) 2025 robot codebase for Team 6045, written in Java using WPILib's command-based framework. The robot is designed for the 2025 Reefscape game and features a swerve drive system with multiple subsystems for game piece manipulation.

## Build & Development Commands

### Build
```bash
./gradlew build
```

### Deploy to Robot
```bash
./gradlew deploy
```

### Run Simulation
```bash
./gradlew simulateJava
```

### Run Tests
```bash
./gradlew test
```

### Clean Build
```bash
./gradlew clean
```

## Architecture

### Command-Based Structure
The robot follows WPILib's command-based architecture:
- **Robot.java**: Entry point, extends `TimedRobot`
- **RobotContainer.java**: Instantiates all subsystems and binds controller inputs
- **Bindings.java**: Contains all controller button mappings and command bindings (separated from RobotContainer for organization)
- **Autos.java**: Registers PathPlanner named commands and manages autonomous routines via SendableChooser

### Subsystems
Located in `src/main/java/frc/robot/subsystems/`:
- **SwerveSubsystem**: Swerve drive using YAGSL (Yet Another Generic Swerve Library) with PathPlanner integration, AprilTag field layout, and custom reef/pole positioning logic
- **ArmSubsystem**: Arm mechanism with PID control and position setpoints
- **ElevatorSubsystem**: Elevator with dual motors, PID control, and limit switches
- **IntakeSubsystem**: Coral intake with distance sensor for game piece detection and Limelight integration for alignment
- **ClimbSubsystem**: Climbing mechanism
- **ClimbWristSubsystem**: Wrist control for climbing
- **LedSubsystem**: LED indicators

### Commands Organization
Commands are organized in subdirectories under `src/main/java/frc/robot/commands/`:
- **ArmCommands/**: Arm open-loop, PID control, and hold commands
- **ElevatorCommands/**: Elevator open-loop, PID control, and hold commands
- **IntakeCommands/**: Intake, outtake, and conditional intake logic
- **AutoScoring/**: Vision-based reef alignment and auto-scoring logic (uses Limelight)
- **swerve/drivebase/**: Swerve drive commands (absolute drive, field-oriented)
- Root level: Combined subsystem commands like `PIDArmAndElevator`

### Constants
`Constants.java` contains nested static classes:
- **MotorConstants**: CAN IDs, current limits, and motor speeds
- **ControllerConstants**: Controller ports and joystick deadbands
- **PositionConstants**: All arm/elevator setpoints for game piece scoring levels (L1-L4, algae, barge, etc.) with a `Setpoints` enum
- **SwerveConstants**: Robot mass, loop time, and max speed
- **AutoScoreConstants**: Reef face and pole positions as Pose2d objects, coral sensor thresholds, and auto-score offsets

### Key Integration Points

#### PathPlanner Integration
- Auto routines are defined in `src/main/deploy/pathplanner/autos/`
- Named commands registered in `Autos.java` constructor
- Swerve configuration files in `src/main/deploy/swerve/neo/`
- PathPlanner uses RobotConfig from GUI settings for trajectory generation

#### YAGSL (Swerve Library)
- SwerveSubsystem loads configuration from `src/main/deploy/swerve/neo/` directory
- Module configs: `frontleft.json`, `frontright.json`, `backleft.json`, `backright.json`
- Controller properties: `controllerproperties.json`
- Custom methods: `getNearestPole()`, `driveToFirstAutoScorePose()`, `driveToSecondAutoScorePose()` for auto-scoring

#### Vision/Limelight
- Limelight name: "limelight-sabre" (see `Constants.LIMELIGHT`)
- IntakeSubsystem has coral detection via distance sensor and alignment offset calculations
- AlignToReefTagRelative command uses Limelight for vision-based alignment
- AprilTag field layout: `AprilTagFields.k2025ReefscapeWelded`

### Vendor Dependencies
Located in `vendordeps/`:
- Phoenix5 & Phoenix6 (CTRE motor controllers)
- REVLib (REV Robotics)
- PathplannerLib
- YAGSL (yagsl-2025.7.2.json)
- Limelight (via maple-sim.json)
- Playing With Fusion, Redux, Studica, ThriftyLib

## Common Development Patterns

### Creating a New Setpoint
1. Add position constants in `PositionConstants` class
2. Add enum value to `PositionConstants.Setpoints`
3. Update `PIDArmAndElevator` command switch statement to handle new setpoint
4. Add controller binding in `Bindings.java` if needed

### Adding Auto Commands
1. Create command in appropriate command package
2. Register in `Autos.java` using `NamedCommands.registerCommand()`
3. Use in PathPlanner GUI by referencing the registered name
4. Add auto to chooser using `autoChooser.addOption()`

### Controller Bindings
- Driver controller (port 0): Primary drive control and intake
- Operator controller (port 1): Mechanism control and setpoints
- Test controller (port 2): Vision testing and auto-score testing
- Bindings separated into sections in `Bindings.java` for clarity

### PID Commands Pattern
Most subsystems use paired commands:
- `PID[Subsystem]Command`: Runs PID to setpoint
- `StopPID[Subsystem]Command`: Stops PID control
- `Hold[Subsystem]`: Default command that holds position
- Combined commands like `PIDArmAndElevator` coordinate multiple subsystems

### Team Number
Team 6045 - configured in `.wpilib/wpilib_preferences.json`

## Important Notes

- The elevator has a bottom limit switch that automatically zeros the encoder (see `ElevatorSubsystem`)
- Arm positions use a "sketchy offset" (`kSketchyOffset = 0.33`) for convenient zero point positioning
- Swerve heading correction is disabled by default (set in `SwerveSubsystem`)
- Barge command uses conditional intake based on arm position threshold
- Alliance color affects auto path mirroring (PathPlanner handles this automatically)
- Red alliance robots get 180-degree rotation offset when zeroing gyro
