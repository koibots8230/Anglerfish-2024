package com.koibots.robot.subsystems.vision;

import com.koibots.robot.Constants.VisionConstants;
import static com.koibots.robot.subsystems.Subsystems.Swerve;

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
            double distance = Math.tan(pitches[i].value * -1) * VisionConstants.NOTE_CAMERA_POSE.getZ();//opposite(?)
            double x = Math.cos(yaws[i].value) * distance;//hypotenuse(?)
            double y = Math.sin(yaws[i].value) * distance;//talked to (ben?) and flipped the variables

            x = x - VisionConstants.NOTE_CAMERA_POSE.getX();
            y = y - VisionConstants.NOTE_CAMERA_POSE.getY();

            double hypotenuse = Math.sqrt((Math.pow(x, 2) + Math.pow(y, 2)));
            double note_angle = Swerve.get().getGyroAngle().getRadians() + yaws[i].value; //radians are weird, this is probably wrong

            if(controller.getAButton() == true){
                if(founder[i].value == true){//i hope that using i again won't break this???
                    System.out.println("Note is detected" + hypotenuse + "meters away");//meters is just an assumption right now
                    
                    if (yaws[i].value < 0){
                        System.out.println("Note is on left");
                    } else if (yaws[i].value > 0) {
                        System.out.println ("Note is on right");}//i may be imagining this wrong...
                    else {
                        System.out.println("Note not detected");
                    }
                }
            }
        }
    }
}
