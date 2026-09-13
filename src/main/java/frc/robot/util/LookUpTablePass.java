package frc.robot.util;

public class LookUpTablePass {
    private static final double MAX_HOOD_ROTATIONS = 5.7;
    private static final double MAX_HOOD_ANGLE_DEGREES = 35.0;
    private static final double HOOD_DEGREES_PER_ROTATION =
        MAX_HOOD_ANGLE_DEGREES / MAX_HOOD_ROTATIONS;

    /**
     * Represents a single entry in the lookup table with distance and corresponding values
     */
    private static class LookUpTableEntryPass {
        final double distance;
        final double hoodpiv;
        final double rpm;

        public LookUpTableEntryPass(double distance, double hoodpiv, double rpm) {
            this.distance = distance;
            this.hoodpiv = hoodpiv;
            this.rpm = rpm;
        }
    }

    /**
     * Lookup table entries for passing.
     *
     * These are spaced at 0.32 m increments and biased a little higher than the
     * normal shot table so the pass arc carries more clearance through midfield.
     * Hood values are stored in degrees after converting the measured hood encoder
     * rotations with hoodRotations * (35.0 / 5.7).
     */
    private static final LookUpTableEntryPass[] LOOKUP_TABLE = {
        // Format: distance meters, hood setpoint (raw rotations, same units as normal LookUpTable), shooter RPM
        new LookUpTableEntryPass(1.448, 3.35, 2520),
        new LookUpTableEntryPass(1.768, 3.60, 2600),
        new LookUpTableEntryPass(2.088, 3.85, 2680),
        new LookUpTableEntryPass(2.408, 4.12, 2770),
        new LookUpTableEntryPass(2.728, 4.38, 2860),
        new LookUpTableEntryPass(3.048, 4.62, 2960),
        new LookUpTableEntryPass(3.368, 4.86, 3070),
        new LookUpTableEntryPass(3.688, 5.06, 3180),
        new LookUpTableEntryPass(4.008, 5.24, 3290),
        new LookUpTableEntryPass(4.328, 5.38, 3410),
        new LookUpTableEntryPass(4.648, 5.50, 3530),
        new LookUpTableEntryPass(4.968, 5.58, 3650),
        new LookUpTableEntryPass(5.288, 5.64, 3770),
        new LookUpTableEntryPass(5.608, 5.68, 3890),
        new LookUpTableEntryPass(5.928, 5.70, 4190),
        new LookUpTableEntryPass(6.248, 5.70, 4310),
        new LookUpTableEntryPass(6.568, 5.70, 4430),
        new LookUpTableEntryPass(6.888, 5.70, 4550),
        new LookUpTableEntryPass(7.208, 5.70, 4670),
        new LookUpTableEntryPass(7.528, 5.70, 4790),
        new LookUpTableEntryPass(7.848, 5.70, 4910),
        new LookUpTableEntryPass(8.168, 5.70, 5030),
        new LookUpTableEntryPass(8.488, 5.70, 5150),
        new LookUpTableEntryPass(8.808, 5.70, 5270),
        new LookUpTableEntryPass(9.128, 5.70, 5390),
        new LookUpTableEntryPass(9.448, 5.70, 5510),
        new LookUpTableEntryPass(9.768, 5.70, 5630),
        new LookUpTableEntryPass(10.000, 5.70, 5720),
        new LookUpTableEntryPass(10.088, 5.70, 5750),
        new LookUpTableEntryPass(10.408, 5.70, 5870),
        new LookUpTableEntryPass(10.728, 5.70, 5990),
        new LookUpTableEntryPass(11.048, 5.70, 6110),
        new LookUpTableEntryPass(11.368, 5.70, 6230),
        new LookUpTableEntryPass(11.688, 5.70, 6350),
        new LookUpTableEntryPass(12.008, 5.70, 6470),
        new LookUpTableEntryPass(12.328, 5.70, 6590),
        new LookUpTableEntryPass(12.648, 5.70, 6710),
        new LookUpTableEntryPass(12.968, 5.70, 6830),
        new LookUpTableEntryPass(13.288, 5.70, 6950),
        new LookUpTableEntryPass(13.608, 5.70, 7070),
        new LookUpTableEntryPass(13.928, 5.70, 7190),
        new LookUpTableEntryPass(14.248, 5.70, 7310),
        new LookUpTableEntryPass(14.568, 5.70, 7430),
        new LookUpTableEntryPass(14.888, 5.70, 7550),
        new LookUpTableEntryPass(15.000, 5.70, 7590),
        new LookUpTableEntryPass(15.208, 5.70, 7670),
        new LookUpTableEntryPass(15.528, 5.70, 7790),
        new LookUpTableEntryPass(15.848, 5.70, 7910),
        new LookUpTableEntryPass(16.168, 5.70, 8030),
        new LookUpTableEntryPass(16.488, 5.70, 8150),
        new LookUpTableEntryPass(16.808, 5.70, 8270),
        new LookUpTableEntryPass(17.128, 5.70, 8390),
        new LookUpTableEntryPass(17.448, 5.70, 8510),
        new LookUpTableEntryPass(17.768, 5.70, 8630),
        new LookUpTableEntryPass(18.088, 5.70, 8750),
        new LookUpTableEntryPass(18.408, 5.70, 8870),
        new LookUpTableEntryPass(18.728, 5.70, 8990),
        new LookUpTableEntryPass(19.048, 5.70, 9110),
        new LookUpTableEntryPass(19.368, 5.70, 9230),
        new LookUpTableEntryPass(19.688, 5.70, 9350),
        new LookUpTableEntryPass(20.000, 5.70, 9470)
    };

    public LookUpTablePass() {
    }

    public static class LookUpTableTest {
        double distance = 0;
        double armpiv = 0;
        double shooterRPS = 0;

        public LookUpTableTest(double hoodpiv, double shooterRPS) {
            this.armpiv = hoodpiv;
            this.shooterRPS = shooterRPS;
        }

        public double getAngle() {
            return armpiv;
        }

        public double getRPM() {
            return shooterRPS;
        }
    }

    /**
     * Gets the lookup table output for a given distance using linear interpolation
     * 
     * @param distance The distance to look up (in meters or your preferred unit)
     * @return LookUpTableTest containing interpolated values for arm pivot and shooter speeds
     */
    public static LookUpTableTest LookUpTableOutput(double distance) {
        // Handle edge cases: distance below minimum or above maximum
        if (distance <= LOOKUP_TABLE[0].distance) {
            LookUpTableEntryPass entry = LOOKUP_TABLE[0];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        if (distance >= LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance) {
            LookUpTableEntryPass entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < LOOKUP_TABLE.length - 1; i++) {
            LookUpTableEntryPass lower = LOOKUP_TABLE[i];
            LookUpTableEntryPass upper = LOOKUP_TABLE[i + 1];

            if (distance >= lower.distance && distance <= upper.distance) {
                // Linear interpolation
                double t = (distance - lower.distance) / (upper.distance - lower.distance);

                double interpolatedArmPiv = lerp(lower.hoodpiv, upper.hoodpiv, t);
                double interpolatedShooterRight = lerp(lower.rpm, upper.rpm, t);

                return new LookUpTableTest(interpolatedArmPiv, interpolatedShooterRight);
            }
        }

        // Fallback (should never reach here)
        LookUpTableEntryPass entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
        return new LookUpTableTest(entry.hoodpiv, entry.rpm);
    }

    /**
     * Linear interpolation helper function
     * 
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated value
     */
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static double hoodRotationsToDegrees(double hoodRotations) {
        return hoodRotations * HOOD_DEGREES_PER_ROTATION;
    }
}
