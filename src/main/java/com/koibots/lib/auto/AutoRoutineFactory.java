// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.auto;

import com.google.flatbuffers.Struct;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.Queue;

public class AutoRoutine<T extends Enum<T>> {
    public enum TemplateActionEnum {
        Nil(new WaitCommand(0)::schedule, '-');

        public final Runnable action;
        public final char signal;

        TemplateActionEnum(Runnable action, char signal) {
            this.action = action;
            this.signal = signal;
        }
    }

    record RoutinePoint(Pose2d start, Pose2d end, TemplateActionEnum action) {
    }


    void preview(Field2d field) {

    }

    static SequentialCommandGroup generateRoutine(Queue<Struct> points) {

    }
}
