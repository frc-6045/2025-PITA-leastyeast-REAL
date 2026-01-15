# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) robot project for Team 6045 (Sabre Robotics), built for the 2025 Reefscape game season. It uses WPILib's command-based framework with Java 17.

## Build Commands

```bash
# Build the project
./gradlew build

# Deploy to the robot (requires connection to roboRIO)
./gradlew deploy

# Run simulation with GUI
./gradlew simulateJava

# Run tests
./gradlew test
```

## Architecture

### Core Structure

- **Robot.java**: Entry point, extends TimedRobot. Handles robot lifecycle (init, periodic, autonomous, teleop).
- **RobotContainer.java**: Instantiates all subsystems and sets up autonomous chooser. Configures default commands.
- **Bindings.java**: All controller button bindings are configured here via `InitBindings()` and `configureDrivetrain()`.
- **Constants.java**: All robot constants including CAN IDs, motor speeds, position setpoints, and field positions.
- **Autos.java**: Autonomous mode configuration using PathPlanner. Registers named commands and builds auto chooser.

### Subsystems

| Subsystem | Purpose | Motor Type | Encoder |
|-----------|---------|------------|---------|
| SwerveSubsystem | YAGSL-based swerve drivetrain | NEO motors | Absolute encoders |
| ArmSubsystem | Rotates arm mechanism | SparkFlex | Absolute encoder (via IntakeSubsystem) |
| ElevatorSubsystem | Vertical lift with two motors | SparkFlex (x2) | Relative encoder + limit switches |
| IntakeSubsystem | Coral/algae intake with distance sensor | SparkFlex | Distance sensor (AnalogPotentiometer) |
| FlywheelSubsystem | Velocity-controlled flywheel for shooting | SparkFlex | Relative encoder (velocity) |
| ClimbSubsystem | Climbing mechanism | - | - |
| ClimbWristSubsystem | Climb wrist articulation | - | - |
| LedSubsystem | LED control | - | - |

### Key Patterns

**Position Setpoints**: Arm and elevator positions are defined in `Constants.PositionConstants` with a `Setpoints` enum (INTAKE, HOME, L1-L4, ALGAE_HIGH, ALGAE_LOW, BARGE, LOLLIPOP). The `PIDArmAndElevator` command coordinates both mechanisms.

**Arm Encoder Offset**: The arm uses a "sketchy offset" (`kSketchyOffset = 0.33`) to normalize encoder readings for PID control. See `ArmSubsystem.getSketchyOffsettedPosition()`.

**Arm Limits**: The arm has soft limits (`kArmLimit1`, `kArmLimit2`) to protect the turnbuckle mechanism. These are enforced in `ArmSubsystem.setSpeed()`.

**Elevator Safety**: The elevator enforces limits based on relative encoder position and uses limit switches. Speed is reduced near boundaries.

**Dashboard Tuning**: Subsystems use SmartDashboard for tuning. Initialize values with `putNumber()` in constructor, read with `getNumber()` in periodic methods. The FlywheelSubsystem demonstrates PID caching to avoid updating gains every cycle when values haven't changed.

### Swerve Drive

Uses YAGSL (Yet Another Swerve Drive Library) with configuration files in `src/main/deploy/swerve/neo/`. PathPlanner is configured for autonomous path following.

### Controllers

- Port 0: Driver controller (drivetrain, intake, arm manual, reef alignment)
- Port 1: Operator controller (setpoints, intake, elevator manual, barge toss)
- Port 2: Test controller (auto-scoring testing)

### Auto Scoring

The `AutoScoreCommands` and `AlignToReefTagRelative` classes handle vision-based reef alignment using Limelight (`limelight-sabre`). Pole positions (A-L) and reef faces are defined in `Constants.AutoScoreConstants`.

## PathPlanner

Autonomous paths are stored in `src/main/deploy/pathplanner/`. Named commands are registered in `Autos.java` for use in PathPlanner GUI.
