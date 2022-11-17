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

    boolean mode = true; // True is passthrough, false is sameside

    // Constants
    public static double leftOffset = -0.025;

    public static double pivotMax = 0.97;
    public static double pivotMin = 0.09;
    public static double pivotPassthroughGrabbingPos = pivotMin;
    public static double pivotSamesideGrabbingPos = 0.95;
    public static double pivotScoringPos = 0.7;
    public static double pivotGroundScoringPos = 0.92;

    public static double endMin = 0;
    public static double endMax = 1;
    public static double endPassthroughGrabbingPos = 0.05;
    public static double endPassthroughScoringPos = 0;
    public static double endSamesideGrabbingPos = 0.7;
    public static double endSamesideScoringPos = 0.69;

    public static double clawClosedPos = 0.22;
    public static double clawOpenPos = 0.45;

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        leftPivot = hwmap.get(Servo.class, "leftPivot");
        rightPivot = hwmap.get(Servo.class, "rightPivot");
        end = hwmap.get(Servo.class, "end");
        claw = hwmap.get(Servo.class, "claw");

        leftPivot.setDirection(Servo.Direction.REVERSE);
        end.setDirection(Servo.Direction.REVERSE);

        // Warning: Robot moves on intitialization
        grabPassthrough();
        openClaw();
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
        // We have to offset one of the servos becuase there is no positon on the splines where they are both at the exact same angle
        leftPivot.setPosition(finalPos + leftOffset);
        rightPivot.setPosition(finalPos);
    }
    public double getPivotPos(){
        // Take the pos of the one we didn't offset
        return rightPivot.getPosition();
    }

    public void setEndPos(double pos){
        double finalPos = Utility.clipValue(endMin, endMax, pos);
        end.setPosition(finalPos);
    }
    public double getEndPos(){
        return end.getPosition();
    }

    // All the different positions of the arm
    public void grabPassthrough(){
        setPivotPos(pivotPassthroughGrabbingPos);
        setEndPos(endPassthroughGrabbingPos);
    }
    public void grabSameside(){
        setPivotPos(pivotSamesideGrabbingPos);
        setEndPos(endSamesideGrabbingPos);
    }
    public void scorePassthrough(){
        setPivotPos(pivotScoringPos);
        setEndPos(endPassthroughScoringPos);
    }
    public void scoreSameside(){
        setPivotPos(pivotScoringPos);
        setEndPos(endSamesideScoringPos);
    }
    public void scoreGroundPassthrough() {
        setPivotPos(pivotGroundScoringPos);
        setEndPos(endPassthroughScoringPos);
    }
    public void scoreGroundSameside() {
        setPivotPos(pivotGroundScoringPos);
        setEndPos(endSamesideScoringPos);
    }

    // Use two modes to switch between passthrough and sameside strategies
    public void setMode(boolean mode){
        this.mode = mode;
    }
    public boolean getMode(){
        return mode;
    }

    public void goToGrab(){
        if (mode) grabPassthrough();
        else grabSameside();
    }
    public void goToScore(){
        if (mode) scorePassthrough();
        else scoreSameside();
    }
    public void goToScoreGround(){
        if (mode) scoreGroundPassthrough();
        else scoreGroundSameside();
    }
}


