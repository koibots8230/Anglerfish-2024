package com.koibots.robot.subsystems.LED;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static LEDs m_LEDs = new LEDs();
    private DigitalOutput digitalOutput;
    private SPI spi;
    private DigitalOutput p1;
    private DigitalOutput p2;
    private DigitalOutput p3;
// DO NOT MERGE UNTILL VERIFIED
    public LEDs() {
        p1 = new DigitalOutput(9);
        p2 = new DigitalOutput(8);
        p3 = new DigitalOutput(7);
    }
    public void send_to_rp2040(int sendMe){
        p1.set((sendMe & 1) != 0);
        p2.set((sendMe & 2) != 0 );
        p3.set((sendMe & 4) != 0 );
    }
}
