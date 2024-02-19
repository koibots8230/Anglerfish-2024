// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.autos.AutoCommands.*;
import static com.koibots.robot.subsystems.Subsystems.Intake;
import static com.koibots.robot.subsystems.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Volts;

import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    static GenericHID controller = new GenericHID(0);
    List<Command> autos = new ArrayList<>();

    public void registerAutos() {
        autos.add(new InstantCommand());
        autos.add(new SysID(SysIDMechanism.Elevator, () -> controller.getRawButton(1)));
        autos.add(new SysID(SysIDMechanism.Swerve, () -> controller.getRawButton(1)));
        autos.add(
                new SequentialCommandGroup(
                        B1_A_N1_S1.command, S1_N4_S2.command, S2_N5_S2.command, S2_N6_S2.command));

        SmartDashboard.putString("Auto Routine", "0");
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        Swerve.get()
                .setDefaultCommand(
                        new FieldOrientedDrive(
                                () -> -controller.getRawAxis(1),
                                () -> -controller.getRawAxis(0),
                                () -> -controller.getRawAxis(4),
                                () -> controller.getPOV(),
                                () -> controller.getRawButton(1)));

        Intake.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new InstantCommand(
                                        () -> Intake.get().setVoltage(Volts.of(12)), Intake.get()),
                                new InstantCommand(
                                        () -> Intake.get().setVoltage(Volts.of(0)), Intake.get()),
                                () -> controller.getRawButton(5)));

        // Elevator.get()
        //         .setDefaultCommand(
        //                 new InstantCommand(
        //                         () -> Elevator.get().setPostion(ElevatorConstants.LOAD_POSITION),
        //                         Elevator.get()));
        // PlopperPivot.get()
        //         .setDefaultCommand(
        //                 new InstantCommand(
        //                         () ->
        //                                 PlopperPivot.get()
        //
        // .setPosition(PlopperPivotConstants.LOAD_POSITION),
        //                         PlopperPivot.get()));
    }

    public Command getAutonomousRoutine() {
        return autos.get(Integer.parseInt(SmartDashboard.getString("Auto Routine", "0")));
    }

    public static void rumbleController(double strength) {
        controller.setRumble(RumbleType.kBothRumble, strength);
    }
}
