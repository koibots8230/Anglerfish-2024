package com.koibots.robot.subsystems.LED;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private DigitalOutput digitalOutput;
    private DigitalOutput p1;
    private DigitalOutput p2;
    private DigitalOutput p3;
    private DigitalOutput p4;
// DO NOT MERGE UNTILL VERIFIED
    public LEDs() {
        p1 = new DigitalOutput(13);
        p2 = new DigitalOutput(12);
        p3 = new DigitalOutput(11);
        p4 = new DigitalOutput(10);
    }
    public void send_to_rp2040(int sendMe){
        p1.set((sendMe & 1) != 0);
        p2.set((sendMe & 2) != 0 );
        p3.set((sendMe & 4) != 0 );
        p4.set((sendMe & 8) != 0);
        System.out.println(sendMe);
    }

}
