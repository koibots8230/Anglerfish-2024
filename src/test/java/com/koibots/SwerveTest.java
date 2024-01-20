// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots;

import com.koibots.robot.subsystems.swerve.Swerve;
import org.junit.jupiter.api.Test;

public class SwerveTest {
    Swerve swerve;

    @Test
    void createSwerve() {
        swerve = new Swerve();
    }
}
