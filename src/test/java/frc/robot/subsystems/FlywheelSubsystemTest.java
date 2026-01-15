package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FlywheelConstants;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FlywheelSubsystemTest {
    private FlywheelSubsystem flywheel;

    @BeforeEach
    void setUp() {
        // Initialize HAL for simulation
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();

        flywheel = new FlywheelSubsystem();
    }

    @AfterEach
    void tearDown() {
        if (flywheel != null) {
            flywheel.close();
        }
        SimHooks.resumeTiming();
    }

    @Test
    void testInitialState() {
        // Flywheel should start with 0 target RPM
        assertEquals(0.0, flywheel.getTargetRPM(), "Initial target RPM should be 0");
    }

    @Test
    void testAtSetpointAfterRunToRPM() {
        // Initially, PID controller has no setpoint so atSetpoint() behavior is undefined
        // After running to 0 RPM (which is the actual velocity), we should be at setpoint
        flywheel.runToRPM(0);
        assertTrue(flywheel.atSetpoint(), "Should be at setpoint when target matches actual (0 RPM)");
    }

    @Test
    void testStopResetsState() {
        // Run to some RPM first
        flywheel.runToRPM(3000);
        assertEquals(3000, flywheel.getTargetRPM(), "Target should be set to 3000");

        // Stop should reset target to 0
        flywheel.stop();
        assertEquals(0, flywheel.getTargetRPM(), "Target should be reset to 0 after stop");
    }

    @Test
    void testRunToRPMSetsTarget() {
        flywheel.runToRPM(4500);
        assertEquals(4500, flywheel.getTargetRPM(), "Target RPM should be set correctly");
    }

    @Test
    void testGetTargetRPMFromDashboardReturnsDefault() {
        // SmartDashboard should have been initialized with the default value
        double target = flywheel.getTargetRPMFromDashboard();
        assertEquals(FlywheelConstants.kDefaultTargetRPM, target,
            "Dashboard should return default target RPM");
    }

    @Test
    void testDashboardValuesInitialized() {
        // Verify that dashboard values were initialized in constructor
        assertEquals(FlywheelConstants.kDefaultP,
            SmartDashboard.getNumber("FLYWHEEL P", -1), 0.0001,
            "P gain should be initialized on dashboard");
        assertEquals(FlywheelConstants.kDefaultI,
            SmartDashboard.getNumber("FLYWHEEL I", -1), 0.0001,
            "I gain should be initialized on dashboard");
        assertEquals(FlywheelConstants.kDefaultD,
            SmartDashboard.getNumber("FLYWHEEL D", -1), 0.0001,
            "D gain should be initialized on dashboard");
        assertEquals(FlywheelConstants.kDefaultFF,
            SmartDashboard.getNumber("FLYWHEEL FF", -1), 0.0001,
            "FF gain should be initialized on dashboard");
        assertEquals(FlywheelConstants.kDefaultToleranceRPM,
            SmartDashboard.getNumber("FLYWHEEL Tolerance", -1), 0.0001,
            "Tolerance should be initialized on dashboard");
    }

    @Test
    void testVelocityRPMReturnsValue() {
        // In simulation, the encoder starts at 0
        double velocity = flywheel.getVelocityRPM();
        assertEquals(0.0, velocity, 0.01, "Initial velocity should be 0 in simulation");
    }

    @Test
    void testPeriodicPublishesTelemetry() {
        // Run periodic to publish telemetry
        flywheel.periodic();

        // Verify telemetry was published
        assertTrue(SmartDashboard.containsKey("FLYWHEEL Actual RPM"),
            "Actual RPM should be published");
        assertTrue(SmartDashboard.containsKey("FLYWHEEL At Setpoint"),
            "At Setpoint should be published");
    }

    @Test
    void testMultipleRunToRPMCalls() {
        // Test that multiple calls update the target correctly
        flywheel.runToRPM(1000);
        assertEquals(1000, flywheel.getTargetRPM());

        flywheel.runToRPM(2000);
        assertEquals(2000, flywheel.getTargetRPM());

        flywheel.runToRPM(5000);
        assertEquals(5000, flywheel.getTargetRPM());
    }

    @Test
    void testRunToRPMClampsToMaxRPM() {
        // Target RPM above max should be clamped to max
        flywheel.runToRPM(10000);  // Well above max of 6000
        assertEquals(FlywheelConstants.kMaxRPM, flywheel.getTargetRPM(),
            "Target RPM should be clamped to max RPM");
    }

    @Test
    void testRunToRPMClampsNegativeToZero() {
        // Negative target RPM should be clamped to 0 (forward only)
        flywheel.runToRPM(-1000);
        assertEquals(0, flywheel.getTargetRPM(),
            "Negative target RPM should be clamped to 0");
    }

    @Test
    void testRunToRPMAtMaxBoundary() {
        // Exactly at max should be accepted
        flywheel.runToRPM(FlywheelConstants.kMaxRPM);
        assertEquals(FlywheelConstants.kMaxRPM, flywheel.getTargetRPM(),
            "Target RPM at max should be accepted");
    }

    @Test
    void testRunToRPMAtZeroBoundary() {
        // Exactly at 0 should be accepted
        flywheel.runToRPM(0);
        assertEquals(0, flywheel.getTargetRPM(),
            "Target RPM at 0 should be accepted");
    }
}
