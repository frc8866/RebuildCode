package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;


public final class HubShotSolver {
    private static final double G = 9.80665;
    private static final double kEpsilon = 1e-9;

    private HubShotSolver() {}

    public static class Result {
        public final double angleDeg;
        public final double velocityMps;

        public Result(double angleDeg, double velocityMps) {
            this.angleDeg = angleDeg;
            this.velocityMps = velocityMps;
        }
    }

    public static Result bestShot(
        double distanceM,
        double targetHeightM,
        double shooterHeightM,
        double minAngleDeg,
        double maxAngleDeg,
        double stepDeg,
        double vMaxMps) {
        Result best = null;

        for (double a = minAngleDeg; a <= maxAngleDeg + 1e-9; a += stepDeg) {
            double v = requiredVelocity(distanceM, targetHeightM, shooterHeightM, a);
            if (Double.isNaN(v)) {
                continue;
            }
            if (vMaxMps > 0.0 && v > vMaxMps) {
                continue;
            }
            if (best == null || v < best.velocityMps) {
                best = new Result(a, v);
            }
        }

        return best;
    }

    public static Result shotFromFunnelClearance(
        Pose2d robot,
        Translation3d target,
        double shooterHeightM,
        double distanceAboveFunnelM) {
        double xDist = distanceToTargetM(robot, target);
        if (xDist <= kEpsilon) {
            return null;
        }

        double yDist = target.getZ() - shooterHeightM;
        double g = G;

        double funnelRadius = Constants.FieldConstants.FUNNEL_RADIUS.in(Meters);
        double funnelHeight = Constants.FieldConstants.FUNNEL_HEIGHT.in(Meters);
        double r = funnelRadius;
        double h = funnelHeight + distanceAboveFunnelM;

        double a1 = xDist * xDist;
        double b1 = xDist;
        double d1 = yDist;
        double a2 = -xDist * xDist + (xDist - r) * (xDist - r);
        double b2 = -r;
        double d2 = h;

        double bm = -b2 / b1;
        double a3 = bm * a1 + a2;
        double d3 = bm * d1 + d2;
        if (Math.abs(a3) <= kEpsilon) {
            return null;
        }

        double a = d3 / a3;
        double b = (d1 - a1 * a) / b1;
        double theta = Math.atan(b);
        double cos = Math.cos(theta);
        double denom = 2.0 * a * cos * cos;
        if (denom >= -kEpsilon) {
            return null;
        }

        double v0 = Math.sqrt(-g / denom);
        if (Double.isNaN(v0) || v0 <= 0.0) {
            return null;
        }

        return new Result(Math.toDegrees(theta), v0);
    }

    public static double distanceToTargetM(Pose2d robot, Translation3d target) {
        return robot.getTranslation().getDistance(target.toTranslation2d());
    }

    private static double requiredVelocity(
        double distanceM, double targetHeightM, double shooterHeightM, double angleDeg) {
        double theta = Math.toRadians(angleDeg);
        double dh = targetHeightM - shooterHeightM;
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double term = distanceM * tan - dh;
        double denom = 2.0 * cos * cos * term;
        if (denom <= 0.0) {
            return Double.NaN;
        }
        return Math.sqrt((G * distanceM * distanceM) / denom);
    }

}
