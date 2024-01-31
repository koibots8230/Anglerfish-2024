// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.auto;

import com.google.flatbuffers.Struct;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Queue;

public class AutoRoutine {
    public interface ActionEnum {
        Runnable getAction();
        char getSignal();
    }

    record RoutinePoint(Pose2d start, Pose2d end, ActionEnum action) {
    }

    private Queue<RoutinePoint> routine;
    
    // public Auto

    void preview(Field2d field) {

    }

    SequentialCommandGroup generateRoutine(Queue<RoutinePoint> points) {
        routine = points;
        
        SequentialCommandGroup commandGroup;
        
        while (!points.isEmpty()) {
           // commandGroup.addCommands();
            
        }
        
        //return commandGroup;
        return null;
    }
}
