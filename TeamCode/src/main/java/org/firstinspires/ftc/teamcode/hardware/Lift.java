package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public static double highPos = 20;

    public static PIDCoefficients coeffs = new PIDCoefficients(0,0,0);
    public static double f = 0.15;

    public Lift(HardwareMap hwmap){
        left = new LinearActuator(hwmap, "leftSpool", 5.2, 5.93);
        right = new LinearActuator(hwmap, "rightSpool", 5.2, 5.93);
        left.zero();
        right.zero();

        left.setLimits(minHeight, maxHeight);
        right.setLimits(minHeight, maxHeight);

        right.setReversed();
    }

    public void setCoefficients(PIDCoefficients coeffs){
        left.setCoefficients(coeffs);
        right.setCoefficients(coeffs);
        Lift.coeffs = coeffs;
        left.setfCoefficient(f);
        right.setfCoefficient(f);
    }

    // Methods
    public void setHeight(double height){
        left.setDistance(height);
        right.setDistance(height);
    }
    public void retract(){
        setHeight(retractedPos);
    }
    public void goToLow(){
        setHeight(lowPos);
    }
    public void goToMedium(){
        setHeight(mediumPos);
    }
    public void goToHigh(){
        setHeight(highPos);
    }
    public void goToJunction(int junction){
        switch (junction){
            case 0:
                retract();
                break;
            case 1:
                goToLow();
                break;
            case 2:
                goToMedium();
                break;
            case 3:
                goToHigh();
                break;
        }
    }

    public void update(){
        left.update();
        right.update();
    }
    public void disalayDebug(Telemetry telemetry){
        left.displayDebugInfo(telemetry);
    }
}
