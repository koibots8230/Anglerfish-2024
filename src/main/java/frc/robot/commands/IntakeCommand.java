package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private final Indexer indexersubsystem;
    private final Intake intakesubststem;

    public IntakeCommand(Indexer indexersubsystem, Intake intakesubststem) {
        this.indexersubsystem = indexersubsystem;
        this.intakesubststem = intakesubststem;
    }

    @Override
    public void initialize() {
        intakesubststem.setIntakeVelocity(PIDConstants.INTAKE_SETPOINT);
        indexersubsystem.setIndexerVelocity(PIDConstants.INTAKE_SETPOINT);
    }

    @Override
    public boolean isFinished() {
        return indexersubsystem.indexerNoteDetected();
    }

    @Override
    public void end(boolean interrupted) {
        intakesubststem.setIntakeVelocity(0.0);
        indexersubsystem.setIndexerVelocity(0.0);
    }
}
