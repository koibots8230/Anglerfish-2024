// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.geometry;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class Wheel {
    public Measure<Distance> circumfrence;
    public Measure<Distance> radius;

    public Wheel(Measure<Distance> radius) {
        this.radius = radius;
        circumfrence = radius.times(2 * Math.PI);
    }
}
