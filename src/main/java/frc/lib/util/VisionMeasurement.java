package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionMeasurement {
  public final Pose2d pose;
  public final double timestamp;
  public final double translationStdev;
  public final double rotationStdev;

  public VisionMeasurement(
      Pose2d pose, double timestamp, double translationStdev, double rotationStdev) {
    this.pose = pose;
    this.timestamp = timestamp;
    this.translationStdev = translationStdev;
    this.rotationStdev = rotationStdev;
  }
}
