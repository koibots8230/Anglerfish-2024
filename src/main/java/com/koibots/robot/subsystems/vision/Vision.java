// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.vision;

import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Meters;

import com.koibots.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;

public class Vision extends SubsystemBase {

    private final DoubleArraySubscriber[][] vecSubscribers;
    private final IntegerSubscriber[] idSubscribers;

    private AprilTagFieldLayout layout;

    public Vision() {
        vecSubscribers = new DoubleArraySubscriber[4][2];
        idSubscribers = new IntegerSubscriber[4];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("fisheye");
        for (int a = 0; a < 4; a++) {
            idSubscribers[a] =
                    table.getIntegerTopic(VisionConstants.TOPIC_NAMES[a][2])
                            .subscribe(VisionConstants.ID_DEFAULT_VALUE);
            for (int b = 0; b < 2; b++) {
                vecSubscribers[a][b] =
                        table.getDoubleArrayTopic(VisionConstants.TOPIC_NAMES[a][b])
                                .subscribe(VisionConstants.VECTOR_DEFAULT_VALUE);
            }
        }

        try {
            layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getPath(), "apriltag", "2024-crescendo.json"));
        } catch (IOException e) {
            System.err.println("ERROR: Could not find apriltag field layout!");
        }

        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                layout.setOrigin(
                        new Pose3d(layout.getFieldLength(), 0, 0, new Rotation3d(0, 0, Math.PI)));
            }
        } else {
            System.err.println("ERROR: Could not determine alliance!");
        }
    }

    private Pose2d translateToFieldPose(
            double[] translation, double[] rotation, int tagId, int camera) {
        int count = 0;
        Matrix<N3, N3> rotMatrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                rotMatrix.set(a, b, rotation[count]);
                count++;
            }
        }

        double hypotenuse = Math.hypot(translation[0], translation[2]);

        double hypangle =
                layout.getTagPose(tagId).get().getRotation().toRotation2d().getRadians()
                        - Math.atan(translation[0] / translation[2]);

        Pose2d camPose =
                new Pose2d(
                        layout.getTagPose(tagId).get().getX()
                                + (hypotenuse * Math.cos(hypangle)),
                        layout.getTagPose(tagId).get().getY() + (hypotenuse * Math.sin(hypangle)),
                        new Rotation2d());

        hypotenuse =
                Math.hypot(
                        VisionConstants.CAMERA_POSITIONS[camera].getX(),
                        VisionConstants.CAMERA_POSITIONS[camera].getY());

        hypangle =
                VisionConstants.CAMERA_POSITIONS[camera].getRotation().getRadians()
                        + Swerve.get().getGyroAngle().getRadians()
                        + 90;

        Rotation2d angle = new Rotation2d(Math.PI + VisionConstants.CAMERA_POSITIONS[camera].getRotation().getRadians() + layout.getTagPose(tagId).get().getRotation().getZ() + Math.atan2(-rotMatrix.get(2, 0), Math.sqrt(Math.pow(rotMatrix.get(2, 1), 2) + Math.pow(rotMatrix.get(2, 2), 2))));

        return new Pose2d(
                camPose.getX() + (hypotenuse * Math.cos(hypangle)),
                camPose.getY() + (hypotenuse * Math.sin(hypangle)),
                angle);
    }

    @Override
    public void periodic() {
        for (int a = 0; a < 4; a++) {
            TimestampedDoubleArray tvec = vecSubscribers[a][0].getAtomic();
            TimestampedDoubleArray rvec = vecSubscribers[a][1].getAtomic();
            TimestampedInteger ids = idSubscribers[a].getAtomic();
            if (ids.timestamp != 0) {
                Pose2d pose =
                        translateToFieldPose(
                                tvec.value, rvec.value, (int) ids.value, a);
                if (pose.getY() > 0
                        && pose.getY() < layout.getFieldWidth()
                        && pose.getX() > 0
                        && pose.getX() < layout.getFieldLength()
                        && Math.abs(pose.getX() - Swerve.get().getEstimatedPose().getX())
                               < VisionConstants.MAX_MEASUREMENT_DIFFERENCE.in(Meters)
                        && Math.abs(pose.getY() - Swerve.get().getEstimatedPose().getY())
                               < VisionConstants.MAX_MEASUREMENT_DIFFERENCE.in(Meters)
                               ) {
                    Swerve.get().addVisionMeasurement(pose, ids.timestamp); //TODO: Figure out why timestamp bad
                }
            }
        }
    }
}
