package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Arm {
    Servo leftPivot;
    Servo rightPivot;
    Servo end;
    Servo claw;

    // Constants
    double pivotMax = 1;
    double pivotMin = 0;
    public static double pivotPassthroughGrabbingPos = 0.2;
    public static double pivotSamesideGrabbingPos = 0.5;
    public static double pivotScoringPos = 0.6;
    public static double pivotLowScoringPos = 0.8;

    public static double endMin = 0;
    public static double endMax = 1;
    public static double endPassthroughGrabbingPos = 0.5;
    public static double endPassthroughScoringPos = 0.5;
    public static double endSamesideGrabbingPos = 0.5;
    public static double endSamesideScoringPos = 0.5;

    public static double clawClosedPos = 0.46;
    public static double clawOpenPos = 0.25;

    // For the setState method
    enum armState{
        GRABBING_PASSTHROUGH,
        SCORING_PASSTHROUGH,
        GRABBING_SAMESIDE,
        SCORING_SAMESIDE
    }

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        leftPivot = hwmap.get(Servo.class, "leftPivot");
        rightPivot = hwmap.get(Servo.class, "rightPivot");
        end = hwmap.get(Servo.class, "end");
        claw = hwmap.get(Servo.class, "claw");

        leftPivot.setDirection(Servo.Direction.REVERSE);

        // Warning: robot moves on ititialization
        setState(armState.GRABBING_PASSTHROUGH);
    }

    // Methods for controlling each dof
    public void openClaw(){
        claw.setPosition(clawOpenPos);
    }
    public void closeClaw(){
        claw.setPosition(clawClosedPos);
    }

    public void setPivotPos(double pos){
        double finalPos = Utility.clipValue(pivotMin, pivotMax, pos);
        leftPivot.setPosition(finalPos);
        rightPivot.setPosition(finalPos);
    }

    public void setEndPos(double pos){
        double finalPos = Utility.clipValue(endMin, endMax, pos);
        end.setPosition(finalPos);
    }

    // This is the big method that combines it all in an easy-to-use way
    public void setState(armState state){
        switch(state){
            case GRABBING_PASSTHROUGH:
                setPivotPos(pivotPassthroughGrabbingPos);
                setEndPos(endPassthroughGrabbingPos);
                break;

            case SCORING_PASSTHROUGH:
                setPivotPos(pivotScoringPos);
                setEndPos(endPassthroughScoringPos);
                break;

            case GRABBING_SAMESIDE:
                setPivotPos(pivotSamesideGrabbingPos);
                setEndPos(endSamesideGrabbingPos);
                break;

            case SCORING_SAMESIDE:
                setPivotPos(pivotScoringPos);
                setEndPos(endSamesideScoringPos);
                break;
        }
    }
}
