// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.util;

import com.koibots.robot.Robot;
import edu.wpi.first.wpilibj.RobotBase;

public class FeedforwardConstantsIO {
    public double ks = 0;
    public double kv = 0;
    public double ka = 0;
    public double kg = 0;

    public FeedforwardConstantsIO() {}

    public FeedforwardConstantsIO(double realKs, double realKv, double simKs, double simKv) {
        if (RobotBase.isReal()) {
            this.ks = realKs;
            this.kv = realKv;
        } else {
            this.ks = simKs;
            this.kv = simKv;
        }
    }

    public FeedforwardConstantsIO(
            double realKs,
            double realKv,
            double realKa,
            double realKg,
            double simKs,
            double simKv,
            double simKa,
            double simKg) {
        if (Robot.isReal()) {
            this.ks = realKs;
            this.kv = realKv;
            this.ka = realKa;
            this.kg = realKg;
        } else {
            this.ks = simKs;
            this.kv = simKv;
            this.ka = simKa;
            this.kg = simKg;
        }
    }
}
