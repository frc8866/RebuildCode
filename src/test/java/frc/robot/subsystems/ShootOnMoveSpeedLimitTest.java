package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ShootOnMoveSpeedLimitTest {
    private static final double EPSILON = 1e-9;

    @Test
    void interpolatesSpeedAndClampsAtBothDistanceBounds() {
        assertSpeedLimit(0.0, 0.2);
        assertSpeedLimit(2.0, 0.2);
        assertSpeedLimit(3.5, 0.3);
        assertSpeedLimit(5.0, 0.4);
        assertSpeedLimit(10.0, 0.4);
    }

    @Test
    void capsDiagonalMagnitudeWhilePreservingDirectionAndRotation() {
        ChassisSpeeds result = Superstructure.limitShootOnMoveSpeedForDistance(
            new ChassisSpeeds(-3.0, 4.0, 1.2), 3.5);

        assertEquals(-0.18, result.vxMetersPerSecond, EPSILON);
        assertEquals(0.24, result.vyMetersPerSecond, EPSILON);
        assertEquals(0.3, Math.hypot(result.vxMetersPerSecond, result.vyMetersPerSecond), EPSILON);
        assertEquals(1.2, result.omegaRadiansPerSecond, EPSILON);
    }

    @Test
    void neverAcceleratesSmallInputsAndAllowsStopping() {
        ChassisSpeeds slow = new ChassisSpeeds(0.06, -0.08, -0.5);
        ChassisSpeeds stopped = new ChassisSpeeds(0.0, 0.0, 0.7);

        assertSame(slow, Superstructure.limitShootOnMoveSpeedForDistance(slow, 2.0));
        assertSame(stopped, Superstructure.limitShootOnMoveSpeedForDistance(stopped, 5.0));
    }

    @Test
    void usesNearLimitForInvalidDistance() {
        assertSpeedLimit(Double.NaN, 0.2);
        assertSpeedLimit(Double.POSITIVE_INFINITY, 0.2);
        assertSpeedLimit(Double.NEGATIVE_INFINITY, 0.2);
    }

    private static void assertSpeedLimit(double distanceMeters, double expectedSpeedMps) {
        ChassisSpeeds result = Superstructure.limitShootOnMoveSpeedForDistance(
            new ChassisSpeeds(3.0, 0.0, 0.0), distanceMeters);
        assertEquals(expectedSpeedMps, result.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, result.vyMetersPerSecond, EPSILON);
    }
}
