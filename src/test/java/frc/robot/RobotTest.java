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
 * Unit tests for Robot class AprilTag detection functionality.
 */
public class RobotTest {

    private MockedStatic<LimelightHelpers> mockedLimelightHelpers;
    private ByteArrayOutputStream outputStream;
    private PrintStream originalOut;

    @BeforeEach
    public void setUp() {
        // Mock static methods of LimelightHelpers
        mockedLimelightHelpers = mockStatic(LimelightHelpers.class);

        // Capture System.out for verifying print statements
        outputStream = new ByteArrayOutputStream();
        originalOut = System.out;
        System.setOut(new PrintStream(outputStream));
    }

    @AfterEach
    public void tearDown() {
        // Close the mock
        mockedLimelightHelpers.close();

        // Restore original System.out
        System.setOut(originalOut);
    }

    @Test
    public void testCheckAprilTagDetection_NoTargetDetected() {
        // Arrange: Configure mock to return no valid target
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(false);

        // Create a Robot instance and invoke the check method via reflection
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // Act: Call the private method
            method.invoke(robot);

            // Assert: Verify no output was printed
            String output = outputStream.toString();
            assertTrue(output.isEmpty() || !output.contains("AprilTag"),
                "No output should be printed when no target is detected");

            // Verify that only getTV was called, not the other methods
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getTV(Constants.LIMELIGHT), times(1));
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getFiducialID(anyString()), never());
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getBotPose3d_TargetSpace(anyString()), never());

        } catch (Exception e) {
            fail("Failed to invoke checkAprilTagDetection method: " + e.getMessage());
        }
    }

    @Test
    public void testCheckAprilTagDetection_TargetDetected() {
        // Arrange: Configure mocks for a detected AprilTag
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(7.0); // AprilTag ID 7

        Pose3d mockPose = new Pose3d(
            new Translation3d(1.234, -0.567, 0.089),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Create a Robot instance and invoke the check method via reflection
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // Act: Call the private method
            method.invoke(robot);

            // Assert: Verify output was printed with correct format
            String output = outputStream.toString();
            assertFalse(output.isEmpty(), "Output should be printed when target is detected");
            assertTrue(output.contains("AprilTag ID 7 detected"),
                "Output should contain AprilTag ID");
            assertTrue(output.contains("X=1.234m"),
                "Output should contain X coordinate");
            assertTrue(output.contains("Y=-0.567m"),
                "Output should contain Y coordinate");
            assertTrue(output.contains("Z=0.089m"),
                "Output should contain Z coordinate");

            // Verify theta is calculated and displayed
            // theta = atan2(-0.567, 1.234) = atan2(y, x) ≈ -24.69 degrees
            double expectedTheta = Math.toDegrees(Math.atan2(-0.567, 1.234));
            String expectedThetaStr = String.format("Theta=%.2f°", expectedTheta);
            assertTrue(output.contains(expectedThetaStr),
                "Output should contain theta: " + expectedThetaStr);

            // Verify all expected methods were called
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getTV(Constants.LIMELIGHT), times(1));
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getFiducialID(Constants.LIMELIGHT), times(1));
            mockedLimelightHelpers.verify(() ->
                LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT), times(1));

        } catch (Exception e) {
            fail("Failed to invoke checkAprilTagDetection method: " + e.getMessage());
        }
    }

    @Test
    public void testCheckAprilTagDetection_TagID_1() {
        testWithSpecificTagID(1);
    }

    @Test
    public void testCheckAprilTagDetection_TagID_15() {
        testWithSpecificTagID(15);
    }

    @Test
    public void testCheckAprilTagDetection_TagID_23() {
        testWithSpecificTagID(23);
    }

    @Test
    public void testCheckAprilTagDetection_TagID_100() {
        testWithSpecificTagID(100);
    }

    private void testWithSpecificTagID(int tagId) {
        // Arrange
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn((double) tagId);

        Pose3d mockPose = new Pose3d(
            new Translation3d(0.5, 0.5, 0.5),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Create a Robot instance and invoke the check method
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // Act
            method.invoke(robot);

            // Assert
            String output = outputStream.toString();
            assertTrue(output.contains("AprilTag ID " + tagId + " detected"),
                "Output should contain correct AprilTag ID: " + tagId);

        } catch (Exception e) {
            fail("Failed to invoke checkAprilTagDetection method for tag ID " + tagId + ": " + e.getMessage());
        }
    }

    @Test
    public void testCheckAprilTagDetection_Position_Positive() {
        testWithSpecificPosition(1.234, -0.567, 0.089);
    }

    @Test
    public void testCheckAprilTagDetection_Position_Zero() {
        testWithSpecificPosition(0.0, 0.0, 0.0);
    }

    @Test
    public void testCheckAprilTagDetection_Position_Negative() {
        testWithSpecificPosition(-2.5, 3.7, -1.2);
    }

    @Test
    public void testCheckAprilTagDetection_Position_Large() {
        testWithSpecificPosition(10.123, 20.456, 30.789);
    }

    private void testWithSpecificPosition(double x, double y, double z) {
        // Arrange
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(1.0);

        Pose3d mockPose = new Pose3d(
            new Translation3d(x, y, z),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Create a Robot instance and invoke the check method
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // Act
            method.invoke(robot);

            // Assert
            String output = outputStream.toString();
            assertTrue(output.contains(String.format("X=%.3fm", x)),
                "Output should contain correct X coordinate");
            assertTrue(output.contains(String.format("Y=%.3fm", y)),
                "Output should contain correct Y coordinate");
            assertTrue(output.contains(String.format("Z=%.3fm", z)),
                "Output should contain correct Z coordinate");

            // Verify theta calculation
            double expectedTheta = Math.toDegrees(Math.atan2(y, x));
            String expectedThetaStr = String.format("Theta=%.2f°", expectedTheta);
            assertTrue(output.contains(expectedThetaStr),
                "Output should contain correct theta: " + expectedThetaStr);

        } catch (Exception e) {
            fail("Failed to invoke checkAprilTagDetection method: " + e.getMessage());
        }
    }

    @Test
    public void testCheckAprilTagDetection_ThetaCalculation_Quadrant1() {
        // Test theta in first quadrant (positive x, positive y) - should be positive angle
        testThetaCalculation(1.0, 1.0, 0.0, 45.0);
    }

    @Test
    public void testCheckAprilTagDetection_ThetaCalculation_Quadrant2() {
        // Test theta in second quadrant (negative x, positive y) - should be > 90°
        testThetaCalculation(-1.0, 1.0, 0.0, 135.0);
    }

    @Test
    public void testCheckAprilTagDetection_ThetaCalculation_Quadrant3() {
        // Test theta in third quadrant (negative x, negative y) - should be < -90°
        testThetaCalculation(-1.0, -1.0, 0.0, -135.0);
    }

    @Test
    public void testCheckAprilTagDetection_ThetaCalculation_Quadrant4() {
        // Test theta in fourth quadrant (positive x, negative y) - should be negative angle
        testThetaCalculation(1.0, -1.0, 0.0, -45.0);
    }

    private void testThetaCalculation(double x, double y, double z, double expectedThetaDegrees) {
        // Arrange
        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getTV(Constants.LIMELIGHT))
            .thenReturn(true);

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getFiducialID(Constants.LIMELIGHT))
            .thenReturn(1.0);

        Pose3d mockPose = new Pose3d(
            new Translation3d(x, y, z),
            new Rotation3d()
        );

        mockedLimelightHelpers.when(() ->
            LimelightHelpers.getBotPose3d_TargetSpace(Constants.LIMELIGHT))
            .thenReturn(mockPose);

        // Create a Robot instance and invoke the check method
        Robot robot = new Robot();
        try {
            java.lang.reflect.Method method = Robot.class.getDeclaredMethod("checkAprilTagDetection");
            method.setAccessible(true);

            // Act
            method.invoke(robot);

            // Assert
            String output = outputStream.toString();
            String expectedThetaStr = String.format("Theta=%.2f°", expectedThetaDegrees);
            assertTrue(output.contains(expectedThetaStr),
                String.format("Output should contain theta=%.2f° for position (%.1f, %.1f, %.1f)",
                    expectedThetaDegrees, x, y, z));

        } catch (Exception e) {
            fail("Failed to invoke checkAprilTagDetection method: " + e.getMessage());
        }
    }
}
