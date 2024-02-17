// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;

import com.koibots.robot.autos.AutonomousRegister;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("RobotName", "Swerve Chassis");
        // Logger.addDataReceiver(new WPILOGWriter());

        if (!DriverStation.isFMSAttached()) {
            Logger.addDataReceiver(new NT4Publisher());
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        if (isReal()) {
            LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        }

        Logger.start();

        // Instantiate our RobotContainer.
        // This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        Elevator.get().reset();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        AutonomousRegister.startSelectedAuto();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.configureButtonBindings();
    }

    @Override
    public void disabledInit() {
        Swerve.get().stop();
    }

    @Override
    public void disabledPeriodic() {}
}
