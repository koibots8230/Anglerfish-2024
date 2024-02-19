package com.koibots.robot.subsystems.vision;

import com.koibots.robot.Constants.VisionConstants;
import static com.koibots.robot.subsystems.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedBoolean;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetection extends SubsystemBase{

    DoubleSubscriber pitch;
    DoubleSubscriber yaw;
    BooleanSubscriber found;

    XboxController controller = new XboxController(VisionConstants.XboxControllerport);

    public NoteDetection() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Arducam_OV9782_USB_Camera");

        pitch = table.getDoubleTopic("targetPitch").subscribe(0);
        yaw = table.getDoubleTopic("targetYaw").subscribe(0);
        found = table.getBooleanTopic("hasTarget").subscribe(false);//i need to check for the right publisher thingymagig
    }

    @Override
    public void periodic() {
        TimestampedDouble[] yaws = yaw.readQueue();
        TimestampedDouble[] pitches = pitch.readQueue();
        TimestampedBoolean[] founder = found.readQueue(); //not sure if this needs to be timestamped
        //went from plurals to different word functions for varibles and i think thats so wonderful

        for(int i = 0; i < yaws.length; i++) {
            double distance = Math.tan(Units.degreesToRadians(pitches[i].value) * -1) * VisionConstants.NOTE_CAMERA_POSE.getZ();//also the hypotenuse
            double x = Math.cos(Units.degreesToRadians(yaws[i].value)) * distance;
            double y = Math.sin(Units.degreesToRadians(yaws[i].value)) * distance;//talked to (ben?) and flipped the variables

            x = x - VisionConstants.NOTE_CAMERA_POSE.getX();
            y = y - VisionConstants.NOTE_CAMERA_POSE.getY();

            double hypotenuse = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));//aka distance
            double note_angle = Swerve.get().getGyroAngle().getRadians() + Units.degreesToRadians(yaws[i].value); 
            //i feel like a bully for not using this. should the dirvers know the angle? (do they want to, more of)

            if(controller.getAButton() == true){
                if(founder[i].value == true){
                    double NOTE_X_COORDINATE = Swerve.get().getEstimatedPose().getX() + x;
                    double NOTE_Y_COORDINATE = Swerve.get().getEstimatedPose().getY() + y;
                    Pose2d NOTE_FIELD_COORDINATE = new Pose2d (NOTE_X_COORDINATE, NOTE_Y_COORDINATE, Swerve.get().getEstimatedPose().getRotation());
                    //that rotation might be wrong, but oh well

                    System.out.println("Note is detected" + hypotenuse + "meters away" + NOTE_FIELD_COORDINATE);
                    Logger.recordOutput("NoteDetection/Note1", NOTE_FIELD_COORDINATE);
                    
                    if (yaws[i].value < 0){
                        System.out.println("Note is on left");
                    } else if (yaws[i].value > 0) {
                        System.out.println ("Note is on right");
                    } else {
                        System.out.println("Note not detected");//this should probably print to the driver station...
                    }
                }
            }
        }
    }
}
