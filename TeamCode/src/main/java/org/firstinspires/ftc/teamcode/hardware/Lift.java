package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Lift {
   DcMotor leftSpool;


    // Constants
    double pivotMax = 1;
    double pivotMin = 0;
    public static double pivotHomePos = 0;
    public static double pivotScoringPos = 1;
    public static double clawClosedPos = 0.46;
    public static double clawOpenPos = 0.25;

    public Lift(HardwareMap hwmap){
        // Hardwaremap stuff
        leftSpool = hwmap.get(DcMotor.class, "leftSpool");
        }

    // Methods




    public void setLiftPower(double pos){
        leftSpool.setPower(pos);
    }
}
