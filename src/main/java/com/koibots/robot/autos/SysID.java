// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.sysid.SysIDMechanism;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class SysID extends SequentialCommandGroup {

//     public SysID(SysIDMechanism mechanism, BooleanSupplier nextButton) {
//         SysIdRoutine sysid =
//                 new SysIdRoutine(
//                         new SysIdRoutine.Config(
//                                 null,
//                                 null,
//                                 Seconds.of(6),
//                                 (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
//                         switch (mechanism) {
//                             case Drive -> new SysIdRoutine.Mechanism(
//                                     (voltage) -> Swerve.get().setDriveVoltages(voltage),
//                                     null,
//                                     Swerve.get());
//                             case Turn -> new SysIdRoutine.Mechanism(
//                                     (voltage) -> Swerve.get().setTurnVoltages(voltage),
//                                     null,
//                                     Swerve.get());
//                         });
//         addCommands(
//                 sysid.quasistatic(SysIdRoutine.Direction.kForward),
//                 new WaitCommand(2.5),
//                 sysid.quasistatic(SysIdRoutine.Direction.kReverse),
//                 new WaitCommand(5),
//                 sysid.dynamic(SysIdRoutine.Direction.kForward),
//                 new WaitCommand(2.5),
//                 sysid.dynamic(SysIdRoutine.Direction.kReverse));

//         addRequirements(Swerve.get());
//     }
}
