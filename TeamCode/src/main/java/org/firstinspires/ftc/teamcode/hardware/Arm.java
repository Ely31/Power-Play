package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    Servo leftPivot;
    Servo rightPivot;

    Servo end;

    Servo claw;

    // Constants
    double pivotMax = 1;
    double pivotMin = 0;
    public static double pivotHomePos = 0;
    public static double pivotScoringPos = 1;
    public static double clawClosedPos = 0.46;
    public static double clawOpenPos = 0.25;

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        leftPivot = hwmap.get(Servo.class, "leftPivot");
        rightPivot = hwmap.get(Servo.class, "rightPivot");
        end = hwmap.get(Servo.class, "end");
        claw = hwmap.get(Servo.class, "claw");

        leftPivot.setDirection(Servo.Direction.REVERSE);
    }

    // Methods
    public void openClaw(){
        claw.setPosition(clawOpenPos);
    }
    public void closeClaw(){
        claw.setPosition(clawClosedPos);
    }

    public void setPivotPos(double pos){
        leftPivot.setPosition(pos);
        rightPivot.setPosition(pos);
    }
}
