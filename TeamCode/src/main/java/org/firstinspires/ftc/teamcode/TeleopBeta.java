package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech;
import org.firstinspires.ftc.teamcode.util.TimeUtil;

@Config
@TeleOp
public class TeleopBeta extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    // Hardware
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    ScoringMech scoringMech;
    ElapsedTime clawActuationTimer = new ElapsedTime();

    // Other variables
    boolean prevClawInput = false;
    // Claw fsm enum
    enum GrabbingState {
        OPEN,
        ClOSED,
        ClOSING,
        WAITING_OPEN
    }
    GrabbingState grabbingState = GrabbingState.OPEN;

    boolean scoring = false;
    boolean prevScoringInput = false;

    boolean prevStackIndexUpInput = false;
    boolean prevStackIndexDownInput = false;

    public static double posEditStep = 0.15;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = false;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.2);
        scoringMech = new ScoringMech(hardwareMap);

        waitForStart();
        matchTimer.reset();
        clawActuationTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Relate the max speed of the bot to the height of the lift to prevent tipping
            drivingSpeedMultiplier = 1 - (scoringMech.getLiftHeight() * 0.035);
            // Drive the bot
            drive.driveFieldCentric(
                    gamepad1.left_stick_x * drivingSpeedMultiplier,
                    gamepad1.left_stick_y * drivingSpeedMultiplier,
                    gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                    gamepad1.right_trigger);
            if (gamepad1.share) drive.resetHeading();

            // CLAW CONTROL
            updateClaw(gamepad1.left_bumper);

            // ARM AND LIFT CONTROL
            // Edit things
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross)     scoringMech.setActiveScoringJunction(0);
            if (gamepad1.square)    scoringMech.setActiveScoringJunction(1);
            if (gamepad1.triangle)  scoringMech.setActiveScoringJunction(2);
            if (gamepad1.circle)    scoringMech.setActiveScoringJunction(3);
            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up)   scoringMech.editCurrentLiftPos(posEditStep);
            if (gamepad2.dpad_down) scoringMech.editCurrentLiftPos(-posEditStep);
            // Edit retracted pose for grabbing off the stack using rising edge detectors
            if (gamepad2.triangle && !prevStackIndexUpInput) {
                scoringMech.setStackIndex(scoringMech.getStackIndex() +1);
                scoringMech.setRetractedGrabbingPose(scoringMech.getStackIndex());
            }
            prevStackIndexUpInput = gamepad2.triangle;
            if (gamepad2.cross && !prevStackIndexDownInput) {
                scoringMech.setStackIndex(scoringMech.getStackIndex() -1);
                scoringMech.setRetractedGrabbingPose(scoringMech.getStackIndex());
            }
            prevStackIndexDownInput = gamepad2.cross;

            // Rising edge detector controlling a toggle for the extended state
            if ((gamepad1.left_trigger > 0) && !prevScoringInput) {
                scoring = !scoring;
            }
            // Extend or retract the lift based on this
            if (scoring) scoringMech.score();
            else if (!(scoringMech.getPivotPos() == Arm.pivotPremovePos && grabbingState == GrabbingState.ClOSING)) scoringMech.retract();
            prevScoringInput = (gamepad1.left_trigger > 0);

            // Update the lift so its pid controller runs, very important
            scoringMech.updateLift();


            // Print stuff to telemetry if we want to
            if (debug) {
                telemetry.addData("extended", scoring);
                telemetry.addData("grabbing state", grabbingStateToString());
                telemetry.addData("stack index", scoringMech.getStackIndex());
                scoringMech.displayDebug(telemetry);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", matchTimer.seconds());
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
                telemetry.addLine();
                telemetry.addLine();
                telemetry.addLine("Gamepad 1 controls:");
                telemetry.addLine("Driving: Left stick is translation, right stick x is rotation. Use the right trigger to slow down.");
                telemetry.addLine("calibrate feild-centric with the share button after you point the bot with the claw facing towards you");
                telemetry.addLine();
                telemetry.addLine("Lift and arm: toggle the claw open and closed with the left bumper, and toggle extend/retract with the left trigger");
                telemetry.addLine("Switch levels with buttons cross, square, triangle, and circle.");
                telemetry.addLine("Ground level is cross, levels get higher as you travel clockwise along the four buttons");
                telemetry.addLine();
                telemetry.addLine("Gamepad 2:");
                telemetry.addLine("make fine adjustments to the current height with dpad up and down.");
                telemetry.addLine("The height tweaks made are saved and will persist until the opmode is stopped.");
            }
            telemetry.update();
        } // End of the loop
    }

    // This is by far the most complicated state machine I've ever made
    void updateClaw(boolean input){
        switch(grabbingState){
            case OPEN:
                scoringMech.openClaw();
                clawActuationTimer.reset();
                // If the button is pressed for the first time or the sensor detects a cone, close the claw
                if ((input && !prevClawInput) || scoringMech.getConeStatus()){
                    grabbingState = GrabbingState.ClOSED;
                }
                break;
            case ClOSED:
                scoringMech.closeClaw();
                clawActuationTimer.reset();
                grabbingState = GrabbingState.ClOSING;
                break;
            case ClOSING:
                if (clawActuationTimer.milliseconds() > Arm.clawActuationTime){
                    scoringMech.preMoveV4b();
                }
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.WAITING_OPEN;
                }
                break;
            case WAITING_OPEN:
                scoringMech.openClaw();
                clawActuationTimer.reset();
                // If you press the button again, close it
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.ClOSED;
                }
                // If it stops seeing the cone, go back to the open state, where it starts to look for one again
                if (!scoringMech.getConeStatus()){
                    grabbingState = GrabbingState.OPEN;
                }
                break;
        }
        prevClawInput = input;
    }

    String grabbingStateToString(){
        // I feel like this should be easier
        String output;
        switch (grabbingState){
            case OPEN:
                output = "open";
                break;
            case ClOSED:
                output = "closed";
                break;
            case ClOSING:
                output = "closing";
                break;
            case WAITING_OPEN:
                output = "waiting open";
                break;
            default:
                output = "default";
                break;
        }
        return output;
    }

}
