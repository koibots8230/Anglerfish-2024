// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.koibots.robot.commands.Swerve.TestDrive;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    List<SendableChooser<Boolean>> modules = new ArrayList<SendableChooser<Boolean>>();
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("RobotName", "Swerve Chassis");
        Logger.addDataReceiver(new WPILOGWriter());

        // CameraServer.startAutomaticCapture();

        if (!DriverStation.isFMSAttached()) {
            Logger.addDataReceiver(new NT4Publisher());
            SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        }

        if (isReal()) {
            LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        }

        Logger.start();

        // Instantiate our RobotContainer.
        // and put our autonomous
        // chooser on the dashboard.
        robotContainer = new RobotContainer();
        robotContainer.registerAutos();

        for (int a = 0; a < 4; a++) {
            SendableChooser<Boolean> module = new SendableChooser<>();
            
            module.setDefaultOption("Enabled", true);
            module.addOption("Disabled", false);

            SmartDashboard.putData("Module " + a, module);

            modules.add(a, module);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
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

    @Override
    public void testInit() {
        SmartDashboard.putData(Swerve.get());
        robotContainer.configureTestBinds(modules);
    }
}
