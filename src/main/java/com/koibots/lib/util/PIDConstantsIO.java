// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.util;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class PIDConstantsIO extends PIDConstants {
    public PIDConstantsIO(
            double realKp, double realKi, double realKd, double simKp, double simKi, double simKd) {
        super(
                RobotBase.isReal() ? realKp : simKp,
                RobotBase.isReal() ? realKi : simKi,
                RobotBase.isReal() ? realKd : simKd);
    }
}
