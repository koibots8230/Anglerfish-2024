package com.koibots.robot.autos;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class GetTheHellOutOfThere extends Command {
    
    public GetTheHellOutOfThere() {
        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        Swerve.get().setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(4, Rotation2d.fromDegrees(-80)),
            new SwerveModuleState(4, Rotation2d.fromDegrees(-80)),
            new SwerveModuleState(4, Rotation2d.fromDegrees(-80)),
            new SwerveModuleState(4, Rotation2d.fromDegrees(-80))
        });
    }
}
