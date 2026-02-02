package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.VisionMeasurement;
import frc.robot.Constants.VisionConstants;
import java.io.UncheckedIOException;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Logged
public class Vision extends SubsystemBase {

  @NotLogged private final DoubleArraySubscriber[][] vecSubscribers;
  @NotLogged private final IntegerSubscriber[] idSubscribers;

  @NotLogged private final Supplier<Pose2d> getOdometry;
  @NotLogged private final Supplier<Rotation2d> getGyro;
  @NotLogged private final Consumer<VisionMeasurement> addMeasurement;

  @NotLogged private AprilTagFieldLayout layout;

  private Pose2d pose;

  private final BooleanSupplier isBlue;

  public Vision(
      Supplier<Pose2d> getOdometry,
      Supplier<Rotation2d> getGyro,
      Consumer<VisionMeasurement> addMeasurement,
      BooleanSupplier isBlue) {
    this.getOdometry = getOdometry;
    this.getGyro = getGyro;
    this.addMeasurement = addMeasurement;

    vecSubscribers = new DoubleArraySubscriber[VisionConstants.ACTIVE_CAMERAS][2];
    idSubscribers = new IntegerSubscriber[VisionConstants.ACTIVE_CAMERAS];

    NetworkTable table = NetworkTableInstance.getDefault().getTable("fisheye");

    for (int a = 0; a < VisionConstants.ACTIVE_CAMERAS; a++) {
      IntegerTopic topic = table.getIntegerTopic(VisionConstants.TOPIC_NAMES[a][2]);
      topic.setCached(true);
      idSubscribers[a] =
          topic.subscribe(
              VisionConstants.ID_DEFAULT_VALUE,
              PubSubOption.sendAll(true),
              PubSubOption.keepDuplicates(true));

      for (int b = 0; b < 2; b++) {
        DoubleArrayTopic topic2 = table.getDoubleArrayTopic(VisionConstants.TOPIC_NAMES[a][b]);
        topic2.setCached(true);
        vecSubscribers[a][b] =
            topic2.subscribe(
                VisionConstants.VECTOR_DEFAULT_VALUE,
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(true));
      }
    }
    try {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    } catch (UncheckedIOException e) {
      System.err.println("ERROR: Could not find apriltag field layout!");
    }

    this.isBlue = isBlue;
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
            layout.getTagPose(tagId).get().getX() + (hypotenuse * Math.cos(hypangle)),
            layout.getTagPose(tagId).get().getY() + (hypotenuse * Math.sin(hypangle)),
            new Rotation2d());

    Rotation2d angle =
        new Rotation2d(
            Math.PI
                + VisionConstants.CAMERA_POSITIONS[camera].getRotation().getRadians()
                + layout.getTagPose(tagId).get().getRotation().getZ()
                + Math.atan2(
                    -rotMatrix.get(2, 0),
                    Math.sqrt(
                        Math.pow(rotMatrix.get(2, 1), 2) + Math.pow(rotMatrix.get(2, 2), 2))));

    return new Pose2d(
        camPose.getX()
            + ((-VisionConstants.CAMERA_POSITIONS[camera].getX()
                    * Math.cos(getGyro.get().getRadians() + (isBlue.getAsBoolean() ? 0 : Math.PI)))
                + (VisionConstants.CAMERA_POSITIONS[camera].getY()
                    * Math.sin(
                        getGyro.get().getRadians() + (isBlue.getAsBoolean() ? 0 : Math.PI)))),
        camPose.getY()
            + ((-VisionConstants.CAMERA_POSITIONS[camera].getY()
                    * Math.cos(getGyro.get().getRadians() + (isBlue.getAsBoolean() ? 0 : Math.PI)))
                - (VisionConstants.CAMERA_POSITIONS[camera].getX()
                    * Math.sin(
                        getGyro.get().getRadians() + (isBlue.getAsBoolean() ? 0 : Math.PI)))),
        angle);
  }

  private boolean withinField(Pose2d pose) {
    return pose.getY() > 0
        && pose.getY() < layout.getFieldWidth()
        && pose.getX() > 0
        && pose.getX() < layout.getFieldLength();
  }

  @Override
  public void periodic() {
    for (int a = 0; a < VisionConstants.ACTIVE_CAMERAS; a++) {
      TimestampedDoubleArray[] tvec = vecSubscribers[a][0].readQueue();
      TimestampedDoubleArray[] rmat = vecSubscribers[a][1].readQueue();
      TimestampedInteger[] ids = idSubscribers[a].readQueue();

      if (tvec.length == 0 || rmat.length == 0 || ids.length == 0) {
        continue;
      } else if (tvec[0].value.length == 1) {
        continue;
      }

      while (!(tvec.length == rmat.length && rmat.length == ids.length)) {
        if (tvec.length > rmat.length || tvec.length > ids.length) {
          tvec = Arrays.copyOf(tvec, tvec.length - 1);
        }
        if (rmat.length > tvec.length || rmat.length > ids.length) {
          rmat = Arrays.copyOf(rmat, rmat.length - 1);
        }
        if (ids.length > rmat.length || ids.length > tvec.length) {
          ids = Arrays.copyOf(ids, ids.length - 1);
        }
      }

      for (int b = 0; b < ids.length; b++) {
        if (ids[b].value != 0
            && ids[b].value != 4
            && ids[b].value != 5
            && ids[b].value != 14
            && ids[b].value != 15
            && tvec[b].timestamp == rmat[b].timestamp
            && rmat[b].timestamp == ids[b].timestamp) {
          pose = translateToFieldPose(tvec[b].value, rmat[b].value, (int) ids[b].value, a);
          if (withinField(pose)
              && ((RobotState.isDisabled())
                  ? true
                  : Math.sqrt(
                          Math.pow(pose.getX() - getOdometry.get().getX(), 2)
                              + Math.pow(pose.getY() - getOdometry.get().getY(), 2))
                      < VisionConstants.MAX_MEASUREMENT_DIFFERENCE.in(Meters))) {
            addMeasurement.accept(
                new VisionMeasurement(
                    pose,
                    Microseconds.of(ids[b].serverTime).in(Seconds),
                    (RobotState.isDisabled()
                        ? 0
                        : Math.pow(
                                1 + Math.hypot(tvec[b].value[0], tvec[b].value[2]),
                                VisionConstants.TRANSLATION_STDEV_ORDER)
                            * VisionConstants.TRANSLATION_STDEV_SCALAR),
                    RobotState.isDisabled() ? 0 : VisionConstants.ROTATION_STDEV));
          }
        }
      }
    }
  }
}
