package com.koibots.robot.subsystems.LED;

import java.lang.reflect.Array;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private static LEDs m_LEDs = new LEDs();
    private SPI spi;

    public LEDs() {
        spi = new SPI(SPI.Port.kOnboardCS0);
    }

    public void writeSPI(byte[] bytes) {
        spi.write(bytes, Array.getLength(bytes));
    }

    public static LEDs get() {
        return m_LEDs;
    }
}