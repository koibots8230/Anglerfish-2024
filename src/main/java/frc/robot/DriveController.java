package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.controller;

public class DriveController {
    BooleanSupplier pressedRightTrigger = () -> controller.CONTROLLER.getRightTriggerAxis() >= .15;


    public DriveController(){
        checkRightTrigger();

    }

    public BooleanSupplier checkRightTrigger(){
        return pressedRightTrigger;
    }



}
