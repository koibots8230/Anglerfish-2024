package com.koibots.robot.subsystems.elevator;

import com.koibots.robot.Constants.ElevatorConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOSim implements ElevatorIO {
    
    private DCMotor gearbox = DCMotor.getNEO(2);

    private double appliedVolts;
    
    private ElevatorSim elevator = new ElevatorSim(
        ElevatorConstants.LINEAR_SYS, 
        gearbox, 
        0, 
        Units.inchesToMeters(9), 
        true, 
        0
    );

    private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 10, 0);
    private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
        new MechanismLigament2d("Elevator", elevator.getPositionMeters(), 90)
    );

    public ElevatorIOSim() {
        SmartDashboard.putData("Elevator Sim", m_mech2d);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        elevator.update(0.020);
        m_elevatorMech2d.setLength(elevator.getPositionMeters());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevator.getCurrentDrawAmps())
        );

        inputs.appliedVoltage = appliedVolts;
        inputs.position = elevator.getPositionMeters();
        inputs.leftAmperage = elevator.getCurrentDrawAmps();
        inputs.rightAmperage = elevator.getCurrentDrawAmps();
    }

    @Override 
    public void setVoltage(double volts) {
        volts = MathUtil.clamp(volts, -12, 12);
        appliedVolts = volts;
        elevator.setInput(volts);
    }

    @Override
    public double getPosition() {
        return elevator.getPositionMeters();
    }

    @Override
    public double getVelocity() {
        return elevator.getVelocityMetersPerSecond();
    }
}
