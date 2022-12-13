package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Utility;

@Config
public class AutoScoringMech {
    Lift lift;
    Arm arm;

    ElapsedTime scoringWait = new ElapsedTime();
    ElapsedTime stackGrabbingWait = new ElapsedTime();

    // Stack position constants
    public static double cone0Pos = 0.2;
    public static double cone1Pos = 0.17;
    public static double cone2Pos = 0.15;
    public static double cone3Pos = 0.12;
    public static double cone4Pos = 0.09;
    // Use an array here so we don't have to do a bunch of switch or if == stuff
    double[] stackPositions = {cone0Pos, cone1Pos, cone2Pos, cone3Pos, cone4Pos};

    enum ScoringState{
        EXTENDING,
        WAIT,
        WAIT2,
        WAIT3,
        RETRACTING,
        DONE
    }
    ScoringState currentScoringState = ScoringState.EXTENDING;

    enum StackGrabbingState{
        CREEPING,
        GRABBING,
        LIFTING,
        DONE
    }
    StackGrabbingState currentStackGrabbingState = StackGrabbingState.CREEPING;

    public AutoScoringMech(HardwareMap hwmap){
        lift = new Lift(hwmap);
        arm = new Arm(hwmap);
    }

    public void grab(){
        arm.closeClaw();
    }
    public void release(){
        arm.openClaw();
    }

    public void preMoveV4B(){
        arm.preMoveV4b();
    }

    public void grabOffStack(int coneNumber){
        while (!(currentStackGrabbingState == StackGrabbingState.DONE)){
            switch (currentStackGrabbingState){
                case CREEPING:
                    release(); // That method is poorly named here
                    arm.setPivotPos(stackPositions[coneNumber]);
                    if (arm.coneIsInClaw()){
                        stackGrabbingWait.reset();
                        grab();
                        currentStackGrabbingState = StackGrabbingState.GRABBING;
                    }
                    break;
                case GRABBING:
                    if (stackGrabbingWait.seconds() > 0.5){
                        stackGrabbingWait.reset();
                        preMoveV4B();
                        currentStackGrabbingState = StackGrabbingState.LIFTING;
                    }
                    break;
                case LIFTING:
                    if (stackGrabbingWait.seconds() > 0.5){
                        currentStackGrabbingState = StackGrabbingState.DONE;
                    }
                    break;
            }
        }
    }

    public void score(double height){
        // While it isn't finished scoring, run an FSM
        while (!(currentScoringState == ScoringState.DONE)){
            switch (currentScoringState){
                case EXTENDING:
                    lift.goToHigh();
                    // Move on if the lift is all the way up
                    if (Utility.withinErrorOfValue(lift.getHeight(), Lift.highPos, 0.5)) {
                        arm.scorePassthrough(); // Move the v4b over the junction
                        scoringWait.reset();
                        currentScoringState = ScoringState.WAIT;
                    }
                    break;
                case WAIT:
                    if (scoringWait.seconds() > 1){ // Wait for the v4b to move all the way
                        arm.openClaw(); // Drop the cone
                        scoringWait.reset();
                        currentScoringState = ScoringState.WAIT2;
                    }
                    break;
                case WAIT2:
                    if (scoringWait.seconds() > 0.2){ // Wait for the cone to drop
                        arm.grabPassthrough(); // Move the v4b inside the bot
                        scoringWait.reset();
                        currentScoringState = ScoringState.WAIT3;
                    }
                    break;
                case WAIT3:
                    if (scoringWait.seconds() > 0.5){ // Wait for the v4b to retract all the way
                        currentScoringState = ScoringState.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    lift.retract(); // Bring the lift down
                    // Move on if the lift is all the way down
                    if (Utility.withinErrorOfValue(lift.getHeight(), Lift.retractedPos, 0.5)) {
                        currentScoringState = ScoringState.DONE; // Finish
                    }
                    break;
            }
            // Always update the lift, no matter what state of scoring it's in
            lift.update();
        }
    }

}
