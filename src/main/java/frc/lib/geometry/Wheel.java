package frc.lib.geometry;

import edu.wpi.first.units.measure.Distance;

public class Wheel {
    public Distance circumfrence;
    public Distance radius;

    public Wheel(Distance radius) {
        this.radius = radius;
        circumfrence = radius.times(2 * Math.PI);
    }
}