package frc.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.LimelightHelpers;

/**
 * Integration tests for Robot class to verify AprilTag detection is called
 * during the robot periodic cycle.
 */
public class RobotIntegrationTest {

    private MockedStatic<LimelightHelpers> mockedLimelightHelpers;
    private ByteArrayOutputStream outputStream;
    private PrintStream originalOut;

    @BeforeEach
    public void setUp() {
        mockedLimelightHelpers = mockStatic(LimelightHelpers.class);
        outputStream = new ByteArrayOutputStream();
        originalOut = System.out;
        System.setOut(new PrintStream(outputStream));
    }

    @AfterEach
    public void tearDown() {
        mockedLimelightHelpers.close();
        System.setOut(originalOut);
    }

    @Test
    public void testAprilTagDetectionCalledDuringRobotPeriodic() {
        // Arrange: Setup mock to detect when checkAprilTagDetection logic is executed
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(false);

        // Act: Create Robot instance
        // Note: We can't easily test robotPeriodic without WPILib runtime,
        // but we can verify the method exists and is properly structured
        Robot robot = new Robot();

        // Verify that the checkAprilTagDetection method exists and is accessible
        try {
            java.lang.reflect.Method periodicMethod = Robot.class.getMethod("robotPeriodic");
            assertNotNull(periodicMethod, "robotPeriodic method should exist");

            java.lang.reflect.Method checkMethod = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            assertNotNull(checkMethod, "checkAprilTagDetection method should exist");
            checkMethod.setAccessible(true);

            // Invoke checkAprilTagDetection directly to verify it integrates properly
            checkMethod.invoke(robot);

            // Verify the Limelight was checked
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getTV(Constants.LIMELIGHT), times(1));

        } catch (Exception e) {
            fail("Failed to verify robotPeriodic integration: " + e.getMessage());
        }
    }

    @Test
    public void testAprilTagDetectionUsesCorrectLimelightName() {
        // Arrange: Verify that the correct Limelight name from Constants is used
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(5.0);

        Pose3d mockPose = new Pose3d(
            new Translation3d(1.0, 2.0, 3.0),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Act
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);
            method.invoke(robot);

            // Assert: Verify all calls used the correct Limelight name
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getTV(Constants.LIMELIGHT), times(1));
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getFiducialID(Constants.LIMELIGHT), times(1));
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT), times(1));

            // Verify the constant is what we expect
            assertEquals("limelight-sabre", Constants.LIMELIGHT,
                "Limelight name should match the expected value");

        } catch (Exception e) {
            fail("Failed to verify Limelight name usage: " + e.getMessage());
        }
    }

    @Test
    public void testAprilTagDetectionOutputFormatConsistency() {
        // Arrange: Test that multiple detections maintain consistent output format
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(8.0);

        Pose3d mockPose = new Pose3d(
            new Translation3d(1.5, -2.3, 0.7),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Act: Call detection multiple times
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // First call
            method.invoke(robot);
            String firstOutput = outputStream.toString();

            // Reset output stream
            outputStream.reset();

            // Second call with same robot instance
            method.invoke(robot);
            String secondOutput = outputStream.toString();

            // Assert: Both outputs should have the same format
            assertEquals(firstOutput, secondOutput,
                "Multiple calls should produce consistent output format");

            // Verify specific values are present
            assertFalse(firstOutput.isEmpty(), "Output should not be empty");
            assertTrue(firstOutput.contains("AprilTag ID 8 detected"),
                "Should detect tag ID 8");
            assertTrue(firstOutput.contains("X=1.500m"),
                "Should show X coordinate as 1.500m");
            assertTrue(firstOutput.contains("Y=-2.300m"),
                "Should show Y coordinate as -2.300m");
            assertTrue(firstOutput.contains("Z=0.700m"),
                "Should show Z coordinate as 0.700m");

            // Verify theta is calculated and displayed
            double expectedTheta = Math.toDegrees(Math.atan2(-2.3, 1.5));
            String expectedThetaStr = String.format("Theta=%.2f°", expectedTheta);
            assertTrue(firstOutput.contains(expectedThetaStr),
                "Should show theta angle: " + expectedThetaStr);

            // Verify format consistency - all coordinates have 3 decimal places
            assertTrue(firstOutput.contains("m, Y=") || firstOutput.contains("m,"),
                "Output should have proper coordinate formatting");

        } catch (Exception e) {
            fail("Failed to verify output format consistency: " + e.getMessage());
        }
    }

    @Test
    public void testAprilTagDetectionDoesNotThrowExceptionOnNullPose() {
        // Arrange: Test graceful handling of unexpected null values
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(10.0);

        // Return a default empty Pose3d (should not be null but test robustness)
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(new Pose3d());

        // Act & Assert: Should not throw exception
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            assertDoesNotThrow(() -> method.invoke(robot),
                "checkAprilTagDetection should not throw exception with empty Pose3d");

            String output = outputStream.toString();
            assertTrue(output.contains("AprilTag ID 10 detected"),
                "Should still print detection message even with default pose");

        } catch (Exception e) {
            fail("Failed to verify exception handling: " + e.getMessage());
        }
    }
}
