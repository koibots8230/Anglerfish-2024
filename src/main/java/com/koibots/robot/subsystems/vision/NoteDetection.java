package com.koibots.robot.subsystems.vision;

import com.koibots.robot.Constants.VisionConstants;
import static com.koibots.robot.subsystems.Subsystems.Swerve;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetection extends SubsystemBase{

    DoubleSubscriber pitch;

    List<Translation2d> notePoses;
    List<Boolean> logsCreated = new LinkedList<>();

    XboxController controller = new XboxController(VisionConstants.XboxControllerport);

    public NoteDetection() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Arducam_OV9782_USB_Camera");

        pitch = table.getDoubleTopic("targetPitch").subscribe(0);
    }

    @Override
    public void periodic() {
        TimestampedDouble[] pitches = pitch.readQueue();

        if (pitches.length > 0) {
            System.out.println("Pitches: " + pitches[0].value);
        }

        notePoses = new LinkedList<>();

        for(int i = 0; i < pitches.length; i++) {
            if (pitches.length == 0) {
                break;
            }
            double distance = Math.tan(Units.degreesToRadians(pitches[i].value) * -1) * VisionConstants.NOTE_CAMERA_POSE.getZ();//also the hypotenuse
            double y = Math.cos(Swerve.get().getEstimatedPose().getRotation().getRadians()) * distance;
            double x = Math.sin(Swerve.get().getEstimatedPose().getRotation().getRadians()) * distance;

            x = x - (VisionConstants.NOTE_CAMERA_DISTANCE_TO_CENTER * Math.sin(Swerve.get().getEstimatedPose().getRotation().getRadians()));
            y = y - (VisionConstants.NOTE_CAMERA_DISTANCE_TO_CENTER * Math.sin(Swerve.get().getEstimatedPose().getRotation().getRadians()));

            Translation2d notePose = new Translation2d(x, y);
            notePoses.add(notePose);

            Logger.recordOutput("Notes/Note " + (i + 1), new Pose2d(notePose, new Rotation2d()));
            if (logsCreated.size() < i + 1) {
                logsCreated.add(true);
            } else {
                logsCreated.set(i, true);
            }
        }

        for (int i = 0; i < logsCreated.size(); i++) {
            if (!logsCreated.get(i)) {
                Logger.recordOutput("Notes/Note " + (i + 1), new Pose2d());
            } 
        }
    }

    public List<Translation2d> getNotes() {
        return notePoses;
    }
}
