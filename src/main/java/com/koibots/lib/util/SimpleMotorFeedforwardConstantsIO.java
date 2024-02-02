// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.util;

import edu.wpi.first.wpilibj.RobotBase;

public class SimpleMotorFeedforwardConstantsIO {
    public double ks;
    public double kv;
    public double ka;

    public SimpleMotorFeedforwardConstantsIO(
            double realKs, double realKv, double simKs, double simKv) {
        if (RobotBase.isReal()) {
            this.ks = realKs;
            this.kv = realKv;
        } else {
            this.ks = simKs;
            this.kv = simKv;
        }
    }
}
