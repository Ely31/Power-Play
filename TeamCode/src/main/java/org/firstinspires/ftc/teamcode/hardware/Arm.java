package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Arm {
    Servo leftPivot;
    Servo rightPivot;
    Servo end;
    Servo claw;

    boolean mode = true; // True is passthrough, false is sameside
    boolean clawState = false;

    // Constants
    public static double leftOffset = -0.08;

    public static double pivotMax = 1;
    public static double pivotMin = 0.09;
    public static double pivotPassthroughGrabbingPos = pivotMin;
    public static double pivotSamesideGrabbingPos = pivotPassthroughGrabbingPos; // Scuffed hack at the tournament
    public static double pivotScoringPos = 1;
    public static double pivotGroundScoringPos = 1;

    public static double endMin = 0;
    public static double endMax = 1;
    public static double endPassthroughGrabbingPos = 0.45;
    public static double endPassthroughScoringPos = 0.43;
    public static double endSamesideGrabbingPos = 0.7;
    public static double endSamesideScoringPos = 0;

    public static double clawClosedPos = 0.2;
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

    public void setClawState(boolean state){
        clawState = state;
    }
    public boolean getClawState(){
        return clawState;
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

    public void update(){
        if (clawState) openClaw();
        else closeClaw();
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addData("Left pivot",leftPivot.getPosition());
        telemetry.addData("Right pivot",rightPivot.getPosition());
        telemetry.addData("End",end.getPosition());
        telemetry.addData("Claw",claw.getPosition());
        telemetry.addData("Mode",mode);
    }
}


