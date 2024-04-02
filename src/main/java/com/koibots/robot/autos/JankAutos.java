// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class JankAutos {

    public Command SP_GetTheHellOuttaThere() {
        return new SequentialCommandGroup(AutoActions.SP(), new GetTheHellOutOfThere());
    }

    public Command SP() {
        return AutoActions.SP();
    }

    public Command SP_LS() {
        return new SequentialCommandGroup(AutoActions.SP(), AutoActions.LS());
    }

    public Command LS() {
        return AutoActions.LS();
    }

    public Command SP_PC_SC() { // center
        return new SequentialCommandGroup(AutoActions.SP(), AutoActions.PC(), AutoActions.SC());
    }

    public Command SP_PL_SL() { // left
        return new SequentialCommandGroup(AutoActions.SP(), AutoActions.PL(), AutoActions.SL());
    }

    public Command SP_PR_SR() { // right
        return new SequentialCommandGroup(AutoActions.SP(), AutoActions.PR(), AutoActions.SR());
    }

    public Command SA_PL_SL() { // amp, left
        return new SequentialCommandGroup(AutoActions.SA(), AutoActions.PL(), AutoActions.SL());
    }

    public Command SA_PL_SA() { // fully amp
        return new SequentialCommandGroup(AutoActions.SA(), AutoActions.PL(), AutoActions.SA());
    }

    public Command SP_PC_SC_PL_SL() { // center, left
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PL(),
                AutoActions.SL());
    }

    public Command SP_PC_SC_PR_SR() { // center, right
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PR(),
                AutoActions.SR());
    }

    public Command SP_PC_SC_PR_SC() { // center, right
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PR(),
                AutoActions.SC());
    }

    public Command SP_PR_SC_PC_SC() { // right, center
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PR(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC());
    }

    public Command SP_PR_SC_PL_SL() { // right, left   (  >:(  )
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PR(),
                AutoActions.SC(),
                AutoActions.PL(),
                AutoActions.SL());
    }

    public Command SP_PL_SC_PC_SC() { // left, center
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC());
    }

    public Command SP_PC_SC_PL_SC() { // left, center
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC());
    }

    public Command SP_W_PL_SC_PC_SC() { // wait, left, center
        return new SequentialCommandGroup(
                AutoActions.SP(),
                new WaitCommand(4),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC());
    }

    public Command SP_PL_SC_PR_SR() { // left, right
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PR(),
                AutoActions.SR());
    }

    public Command SP_PC_SC_PL_SA() { // center, left to amp
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PL(),
                AutoActions.SA());
    }

    public Command SP_PR_SC_PL_SA() { // right, center to amp
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PR(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SA());
    }

    public Command SP_PL_SC_PC_SA() { // left, center to amp
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SA());
    }

    public Command SA_PL_SA_PC_SC() { // fully amp, center
        return new SequentialCommandGroup(
                AutoActions.SA(),
                AutoActions.PL(),
                AutoActions.SA(),
                AutoActions.PC(),
                AutoActions.SC());
    }

    // ================================== 4 Piece ==================================

    public Command
            SP_PL_SC_PC_SC_PR_SC() { // get + shoot all of the ground notes from left to right
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PL(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PR(),
                AutoActions.SC());
    }

    public Command
            SP_PR_SC_PC_SC_PL_SC() { // get + shoot all of the ground notes from right to left
        return new SequentialCommandGroup(
                AutoActions.SP(),
                AutoActions.PR(),
                AutoActions.SC(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PL(),
                AutoActions.SC());
    }

    public Command SA_PL_SA_PC_SC_PR_SC() { // fully amp & score 2 speaker
        return new SequentialCommandGroup(
                AutoActions.SA(),
                AutoActions.PL(),
                AutoActions.SA(),
                AutoActions.PC(),
                AutoActions.SC(),
                AutoActions.PR(),
                AutoActions.SC());
    }
}
