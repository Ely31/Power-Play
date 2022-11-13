package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.actuators.LinearActuator;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Lift {
    LinearActuator left;
    LinearActuator right;

    // Measurements are in inches
    public static double maxHeight = 30;
    public static double minHeight = 0;

    public static double retractedPos = 0;
    public static double groundPos = 0;
    public static double lowPos = 0;
    public static double mediumPos = 10;
    public static double highPos = 35;

    public Lift(HardwareMap hwmap){
        left = new LinearActuator(hwmap, "leftLift", 5.2, 5.93);
        right = new LinearActuator(hwmap, "rightLift", 5.2, 5.93);

        left.setLimits(minHeight, maxHeight);

        right.setReversed();
    }

    // Methods
    public void setHeight(double height){
        left.setDistance(height);
        right.setDistance(height);
    }

    public void update(){
        left.update();
        right.update();
    }
}
