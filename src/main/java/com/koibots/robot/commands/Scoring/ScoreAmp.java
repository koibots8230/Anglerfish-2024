// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.commands.Scoring;

import com.koibots.lib.geometry.PloppervatorPosition;
import com.koibots.robot.commands.Ploppervator.RunPlopper;
import com.koibots.robot.commands.Ploppervator.SetPloppervatorPosition;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmp extends SequentialCommandGroup {

    public ScoreAmp() {
        addCommands(
                // TODO: Path to amp command here
                new SetPloppervatorPosition(PloppervatorPosition.Amp), new RunPlopper(false));
    }
}
