package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public static Result bestShotWithRobotSpeed(
        double distanceM,
        double targetHeightM,
        double shooterHeightM,
        double robotSpeedTowardTargetMps,
        double minAngleDeg,
        double maxAngleDeg,
        double stepDeg,
        double vMaxMps) {
        Result best = null;

        for (double a = minAngleDeg; a <= maxAngleDeg + 1e-9; a += stepDeg) {
            double v = requiredVelocityWithRobotSpeed(
                distanceM, targetHeightM, shooterHeightM, robotSpeedTowardTargetMps, a);
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

    public static Result iterativeMovingShotFromFunnelClearance(
        Pose2d robot,
        ChassisSpeeds fieldSpeeds,
        Translation3d target,
        double shooterHeightM,
        double distanceAboveFunnelM,
        int iterations) {
        Result shot = shotFromFunnelClearance(robot, target, target, shooterHeightM, distanceAboveFunnelM);
        if (shot == null) {
            return null;
        }

        double distance = distanceToTargetM(robot, target);
        double timeOfFlight = calculateTimeOfFlightSeconds(shot, distance);
        if (Double.isNaN(timeOfFlight)) {
            return null;
        }

        Translation3d predictedTarget = target;
        for (int i = 0; i < iterations; i++) {
            predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
            shot = shotFromFunnelClearance(robot, target, predictedTarget, shooterHeightM, distanceAboveFunnelM);
            if (shot == null) {
                return null;
            }
            distance = distanceToTargetM(robot, predictedTarget);
            timeOfFlight = calculateTimeOfFlightSeconds(shot, distance);
            if (Double.isNaN(timeOfFlight)) {
                return null;
            }
        }

        return shot;
    }

    public static Result shotFromFunnelClearance(
        Pose2d robot,
        Translation3d actualTarget,
        Translation3d predictedTarget,
        double shooterHeightM,
        double distanceAboveFunnelM) {
        double xDist = distanceToTargetM(robot, predictedTarget);
        if (xDist <= kEpsilon) {
            return null;
        }

        double yDist = predictedTarget.getZ() - shooterHeightM;
        double g = G;

        double actualDistance = distanceToTargetM(robot, actualTarget);
        if (actualDistance <= kEpsilon) {
            return null;
        }

        double funnelRadius = Constants.FieldConstants.FUNNEL_RADIUS.in(Meters);
        double funnelHeight = Constants.FieldConstants.FUNNEL_HEIGHT.in(Meters);
        double r = funnelRadius * xDist / actualDistance;
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

    public static Translation3d predictTargetPos(
        Translation3d target, ChassisSpeeds fieldSpeeds, double timeOfFlightSeconds) {
        double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlightSeconds;
        double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlightSeconds;
        return new Translation3d(predictedX, predictedY, target.getZ());
    }

    public static double distanceToTargetM(Pose2d robot, Translation3d target) {
        return robot.getTranslation().getDistance(target.toTranslation2d());
    }

    private static double calculateTimeOfFlightSeconds(Result shot, double distanceM) {
        double theta = Math.toRadians(shot.angleDeg);
        double denom = shot.velocityMps * Math.cos(theta);
        if (denom <= kEpsilon) {
            return Double.NaN;
        }
        return distanceM / denom;
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

    private static double requiredVelocityWithRobotSpeed(
        double distanceM,
        double targetHeightM,
        double shooterHeightM,
        double robotSpeedTowardTargetMps,
        double angleDeg) {
        double theta = Math.toRadians(angleDeg);
        double dh = targetHeightM - shooterHeightM;
        double c = Math.cos(theta);
        double s = Math.sin(theta);
        double vr = robotSpeedTowardTargetMps;

        double a = dh * c * c - distanceM * s * c;
        double b = 2.0 * dh * vr * c - distanceM * s * vr;
        double c0 = dh * vr * vr + (G * distanceM * distanceM) / 2.0;

        double u;
        if (Math.abs(a) < kEpsilon) {
            if (Math.abs(b) < kEpsilon) {
                return Double.NaN;
            }
            u = -c0 / b;
            if (u <= 0.0 || (vr + u * c) <= 0.0) {
                return Double.NaN;
            }
            return u;
        }

        double disc = b * b - 4.0 * a * c0;
        if (disc < 0.0) {
            return Double.NaN;
        }

        double sqrtDisc = Math.sqrt(disc);
        double u1 = (-b + sqrtDisc) / (2.0 * a);
        double u2 = (-b - sqrtDisc) / (2.0 * a);
        double best = Double.NaN;

        if (u1 > 0.0 && (vr + u1 * c) > 0.0) {
            best = u1;
        }
        if (u2 > 0.0 && (vr + u2 * c) > 0.0) {
            if (Double.isNaN(best) || u2 < best) {
                best = u2;
            }
        }

        return best;
    }
}
