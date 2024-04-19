package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;

public class ShooterCommand extends Command {
    private final ShooterCommand shooterSubsystem;
    

    public ShooterCommand(ShooterCommand shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute(){

    }


    @Override
    public boolean isFinished(){
        return false;
    }

}
