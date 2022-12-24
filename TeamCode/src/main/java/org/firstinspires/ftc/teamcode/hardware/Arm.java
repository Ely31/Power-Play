package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.sensors.RevColorSensor;
import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class Arm {
    Servo leftPivot;
    Servo rightPivot;
    Servo end;
    Servo claw;
    ColorSensor clawSensor;

    boolean clawState = false; // True is closed

    // Constants
    public static double leftOffset = 0.005;

    public static double pivotMax = 0.97;
    public static double pivotMin = 0.04;
    public static double pivotPassthroughGrabbingPos = pivotMin;
    public static double pivotScoringPos = 0.7;
    public static double pivotGroundScoringPos = 0.92;

    public static double endMin = 0;
    public static double endMax = 1;
    public static double endPassthroughGrabbingPos = 0.53;
    public static double endPassthroughScoringPos = 0.45;

    public static double clawClosedPos = 0.96;
    public static double clawOpenPos = 0.45;
    public static double clawActuationTime = 400; // In milliseconds

    public static double sensorThreshold = 800;

    public Arm(HardwareMap hwmap){
        // Hardwaremap stuff
        leftPivot = hwmap.get(Servo.class, "leftPivot");
        rightPivot = hwmap.get(Servo.class, "rightPivot");
        end = hwmap.get(Servo.class, "end");
        claw = hwmap.get(Servo.class, "claw");
        clawSensor = hwmap.get(ColorSensor.class, "clawSensor");

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
    public void scorePassthrough(){
        setPivotPos(pivotScoringPos);
        setEndPos(endPassthroughScoringPos);
    }
    public void scoreGroundPassthrough() {
        setPivotPos(pivotGroundScoringPos);
        setEndPos(endPassthroughScoringPos);
    }

    public void preMoveV4b(){
        setPivotPos(0.48);
    }


    public boolean coneIsInClaw(){
        return clawSensor.alpha() > sensorThreshold;
    }

    public void update(){
        if (!clawState) openClaw();
        else closeClaw();
    }

    public void displayDebug(Telemetry telemetry){
        telemetry.addData("Left pivot pos",leftPivot.getPosition());
        telemetry.addData("Right pivot pos",rightPivot.getPosition());
        telemetry.addData("End pos",end.getPosition());
        telemetry.addData("Claw pos",claw.getPosition());
        telemetry.addData("Claw closed", getClawState());
        telemetry.addData("Claw sensor val", clawSensor.alpha());
        telemetry.addData("Cone in claw", coneIsInClaw());
    }
}
