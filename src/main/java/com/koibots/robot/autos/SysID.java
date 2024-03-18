// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.*;

import com.koibots.lib.sysid.SysIDMechanism;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SysID extends SequentialCommandGroup {

    public SysID(SysIDMechanism mechanism, BooleanSupplier nextButton) {
        SysIdRoutine sysid =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                        switch (mechanism) {
                            case Swerve -> new SysIdRoutine.Mechanism(
                                    (voltage) -> Swerve.get().setDriveVoltages(voltage),
                                    null,
                                    Swerve.get());
                        });
        addCommands(
                sysid.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitUntilCommand(nextButton),
                sysid.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitUntilCommand(nextButton),
                sysid.dynamic(SysIdRoutine.Direction.kForward),
                new WaitUntilCommand(nextButton),
                sysid.dynamic(SysIdRoutine.Direction.kForward));

        addRequirements(
                switch (mechanism) {
                    case Swerve -> Swerve.get();
                });
    }
}
