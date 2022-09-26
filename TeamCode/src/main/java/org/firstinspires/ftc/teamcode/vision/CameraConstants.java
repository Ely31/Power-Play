package org.firstinspires.ftc.teamcode.vision;

public class CameraConstants {
    public static final int cameraWidth = 320, cameraHeight = 240;

    // Haven't calibrated the vertical fov yet, migt be totally wrong
    public static final double horizontalFOV = 42, verticalFOV = 42;

    public static final double
        pixelsPerHorizontalDegree = horizontalFOV / cameraWidth,
        pixelsPerVerticalDegree = verticalFOV / cameraHeight;
}
