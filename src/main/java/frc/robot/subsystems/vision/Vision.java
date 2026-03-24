// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.LinkedList;
import java.util.List;
import java.util.function.IntPredicate;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final IntPredicate poseAcceptanceFilter;
    private final BooleanPublisher hasTargetPublisher =
            NetworkTableInstance.getDefault().getBooleanTopic("/Vision/HasTarget").publish();
    private static final double kLogPeriodSeconds = 0.1;
    private double lastLogTimeSeconds = 0.0;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this(consumer, cameraIndex -> true, io);
    }

    public Vision(VisionConsumer consumer, IntPredicate poseAcceptanceFilter, VisionIO... io) {
        this.consumer = consumer;
        this.poseAcceptanceFilter = poseAcceptanceFilter;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public TargetObservation[] getTargetObservations(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= inputs.length) {
            return new TargetObservation[0];
        }
        return inputs[cameraIndex].targetObservations;
    }

    public boolean hasTarget(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= inputs.length) {
            return false;
        }
        var cameraInputs = inputs[cameraIndex];
        return cameraInputs.targetObservations.length > 0
                || cameraInputs.tagIds.length > 0
                || cameraInputs.poseObservations.length > 0
                || Math.abs(cameraInputs.latestTargetObservation.tx().getRadians()) > 1e-6
                || Math.abs(cameraInputs.latestTargetObservation.ty().getRadians()) > 1e-6;
    }

    @Override
    public void periodic() {
        double nowSeconds = Timer.getFPGATimestamp();
        boolean shouldLog = nowSeconds - lastLogTimeSeconds >= kLogPeriodSeconds;
        boolean anyHasTarget = false;
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            if (shouldLog) {
                Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
            }
            if (inputs[i].targetObservations.length > 0
                    || inputs[i].tagIds.length > 0
                    || inputs[i].poseObservations.length > 0) {
                anyHasTarget = true;
            }
        }
        hasTargetPublisher.set(anyHasTarget);

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();
        BestObservation bestObservation = null;

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                if (!poseAcceptanceFilter.test(cameraIndex)) {
                    robotPosesRejected.add(observation.pose());
                    continue;
                }

                robotPosesAccepted.add(observation.pose());

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                var measurementStdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
                double observationScore = scoreObservation(observation, linearStdDev, angularStdDev);
                if (bestObservation == null || observationScore < bestObservation.score()) {
                    bestObservation = new BestObservation(observation, measurementStdDevs, observationScore);
                }
            }

            if (shouldLog) {
                // Log camera data
                Logger.recordOutput(
                        "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                        tagPoses.toArray(new Pose3d[tagPoses.size()]));
                Logger.recordOutput(
                        "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                        robotPoses.toArray(new Pose3d[robotPoses.size()]));
                Logger.recordOutput(
                        "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                        robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
                Logger.recordOutput(
                        "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                        robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
                allTagPoses.addAll(tagPoses);
                allRobotPoses.addAll(robotPoses);
                allRobotPosesAccepted.addAll(robotPosesAccepted);
                allRobotPosesRejected.addAll(robotPosesRejected);
            }
        }

        if (shouldLog) {
            // Log summary data
            Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
            Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Summary/RobotPosesAccepted",
                    allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Summary/RobotPosesRejected",
                    allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
            lastLogTimeSeconds = nowSeconds;
        }

        if (bestObservation != null) {
            consumer.accept(
                    bestObservation.observation().pose().toPose2d(),
                    bestObservation.observation().timestamp(),
                    bestObservation.visionMeasurementStdDevs());
        }
    }

    private static double scoreObservation(VisionIO.PoseObservation observation, double linearStdDev, double angularStdDev) {
        double angleComponent = Double.isFinite(angularStdDev) ? angularStdDev : 0.0;
        return linearStdDev + angleComponent - (observation.tagCount() * 0.25);
    }

    private record BestObservation(
            VisionIO.PoseObservation observation, Matrix<N3, N1> visionMeasurementStdDevs, double score) {}

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}



