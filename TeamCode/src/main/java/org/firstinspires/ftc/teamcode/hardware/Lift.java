package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.actuators.PIDActuator;

public class Lift {
    PIDActuator left;
    PIDActuator right;

    final double SLIDES_ANGLE = 17; // In degrees
    final double SPOOL_DIAMETER = 1.8937; // In inches
    double INCHES_PER_ROTATION;
    int TICKS_PER_INCH;

    // Measurements are in inches, compensated for the slides angle
    public static double retractedPos = 0;
    public static double groundPos = 0;
    public static double lowPos = 0;
    public static double mediumPos = 10;
    public static double highPos = 35;

    public Lift(HardwareMap hwmap){
        left = new PIDActuator(hwmap, "leftLift", 5.2);
        right = new PIDActuator(hwmap, "rightLift", 5.2);

        // Kinda weird doing ticks per inch in here, but it works
        INCHES_PER_ROTATION = SPOOL_DIAMETER * Math.PI;
        TICKS_PER_INCH = (int) (left.TICKS_PER_REV / INCHES_PER_ROTATION);
    }

    // Methods
    public void setHeight(double height){

    }

}
