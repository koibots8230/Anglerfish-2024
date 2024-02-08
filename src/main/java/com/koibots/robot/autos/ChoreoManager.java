// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

public class ChoreoManager {
    private static final ChoreoManager instance = new ChoreoManager();

    public static ChoreoManager get() {
        return instance;
    }

    private ChoreoManager() {}
}
