package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TeleMecDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.hardware.ScoringMech;
import org.firstinspires.ftc.teamcode.util.TimeUtil;
import org.firstinspires.ftc.teamcode.vision.workspace.JunctionBasedOnHubPipeline;

@Config
@TeleOp
public class Teleop2 extends LinearOpMode {
    // Pre init
    TimeUtil timeUtil = new TimeUtil();
    ElapsedTime matchTimer = new ElapsedTime();
    // Hardware
    TeleMecDrive drive;
    double drivingSpeedMultiplier;
    ScoringMech scoringMech;
    PivotingCamera camera;
    JunctionBasedOnHubPipeline pipeline = new JunctionBasedOnHubPipeline();
    public static PIDCoefficients trackingCoeffs = new PIDCoefficients(0.003,0.0001,0.0001);
    PIDFController trackingController = new PIDFController(trackingCoeffs);

    ElapsedTime clawActuationTimer = new ElapsedTime();
    ElapsedTime pivotActuationTimer = new ElapsedTime();

    // Other variables
    boolean prevClawInput = false;
    // Claw fsm enum
    enum GrabbingState {
        OPEN,
        ClOSED,
        WAITING_OPEN
    }
    GrabbingState grabbingState = GrabbingState.OPEN;

    enum ScoringState{
        RETRACTED,
        PREMOVED,
        SCORING
    }
    ScoringState scoringState = ScoringState.RETRACTED;

    // Stuff for rising edge detectors
    boolean prevScoringInput = false;

    boolean prevStackIndexUpInput = false;
    boolean prevStackIndexDownInput = false;

    boolean trackingJunction = false;
    boolean prevTrackingJunctionInput = false;

    // Lift constants
    public static double liftPosEditStep = 0.15;
    public static double liftRawPowerAmount = -0.1;

    // Telemetry options
    public static boolean debug = true;
    public static boolean instructionsOn = true;

    @Override
    public void runOpMode(){
        // Init
        telemetry.setMsTransmissionInterval(100);
        // Bind hardware to the hardwaremap
        drive = new TeleMecDrive(hardwareMap, 0.2);
        scoringMech = new ScoringMech(hardwareMap);
        camera = new PivotingCamera(hardwareMap, pipeline);
        trackingController.setTargetPosition(0);

        waitForStart();
        matchTimer.reset();
        clawActuationTimer.reset();
        pivotActuationTimer.reset();
        while (opModeIsActive()){
            // Send signals to drivers when endgame approaches
           timeUtil.updateAll(matchTimer.milliseconds(), gamepad1, gamepad2);

            // Relate the max speed of the bot to the height of the lift to prevent tipping
            drivingSpeedMultiplier = 1 - (scoringMech.getLiftHeight() * 0.035);
            // Drive the bot
            // If tracking is on, we're scoring, and the driver isn't trying to turn the bot, hand over control of turning to the camera and pid controller
            if (trackingJunction && (scoringState == ScoringState.SCORING) && (gamepad1.right_stick_x == 0)) {
                drive.driveFieldCentric(
                        gamepad1.left_stick_x * drivingSpeedMultiplier,
                        gamepad1.left_stick_y * drivingSpeedMultiplier,
                        trackingController.update(-pipeline.getJunctionX()),
                        gamepad1.right_trigger);
            } else {
                // Otherwise, drive normally
                drive.driveFieldCentric(
                        gamepad1.left_stick_x * drivingSpeedMultiplier,
                        gamepad1.left_stick_y * drivingSpeedMultiplier,
                        gamepad1.right_stick_x * drivingSpeedMultiplier * 0.8,
                        gamepad1.right_trigger);
            }
            // Manually calibrate feild centric with a button
            if (gamepad1.share) drive.resetHeading();

            // Make the camera point at the current junction height
            camera.setJunction(scoringMech.getActiveScoringJunction());
            // Toggle junction tracking with start
            if (gamepad1.start && !prevTrackingJunctionInput) trackingJunction = !trackingJunction;
            prevTrackingJunctionInput = gamepad1.start;

            // ARM AND LIFT CONTROL
            // Edit things
            // Switch active junction using the four buttons on gamepad one
            if (gamepad1.cross)     scoringMech.setActiveScoringJunction(0);
            if (gamepad1.square)    scoringMech.setActiveScoringJunction(1);
            if (gamepad1.triangle)  scoringMech.setActiveScoringJunction(2);
            if (gamepad1.circle)    scoringMech.setActiveScoringJunction(3);
            // Edit the current level with the dpad on gamepad two
            if (gamepad2.dpad_up)   scoringMech.editCurrentLiftPos(liftPosEditStep);
            if (gamepad2.dpad_down) scoringMech.editCurrentLiftPos(-liftPosEditStep);
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

            // Claw control
            updateClaw(gamepad1.left_bumper);

            // Run the scoring fsm
            switch (scoringState){
                case RETRACTED:
                    scoringMech.retract(pivotActuationTimer.milliseconds());

                    if (grabbingState == GrabbingState.ClOSED){
                        scoringState = ScoringState.PREMOVED;
                    }
                    break;

                case PREMOVED:
                    scoringMech.retractLift();
                    scoringMech.preMoveV4b();

                    if ((gamepad1.left_trigger > 0) && !prevScoringInput) {
                        scoringState = ScoringState.SCORING;
                    }
                    break;

                case SCORING:
                    scoringMech.score();
                    // Retract if you press the retract button
                    if ((gamepad1.left_trigger > 0) && !prevScoringInput) {
                        pivotActuationTimer.reset();
                        scoringState = ScoringState.RETRACTED;
                    }
                    // Or, retract automatically when you drop the cone
                    if (grabbingState == GrabbingState.OPEN){
                        pivotActuationTimer.reset();
                        scoringState = ScoringState.RETRACTED;
                    }
                    break;
            }
            prevScoringInput = (gamepad1.left_trigger > 0);


            // Update the lift so its pid controller runs, very important
            // But, if you press a special key combo, escape pid control and bring the lift down
            // With raw power to fix a lift issue
            if (gamepad2.dpad_left && gamepad2.share){
                scoringMech.setRawLiftPowerDangerous(liftRawPowerAmount);
            }
            else scoringMech.updateLift();


            // Print stuff to telemetry if we want to
            if (debug) {
                telemetry.addData("scoring state", scoringState.name());
                telemetry.addData("active junction", scoringMech.getActiveScoringJunction());
                telemetry.addData("grabbing state", grabbingState.name());
                telemetry.addData("stack index", scoringMech.getStackIndex());
                telemetry.addData("tracking", trackingJunction);
                scoringMech.displayDebug(telemetry);
                telemetry.addData("avg loop time (ms)", timeUtil.getAverageLoopTime());
                telemetry.addData("period", timeUtil.getPeriod());
                telemetry.addData("time", matchTimer.seconds());
            }
            // Someone should be able to learn how to drive without looking at the source code
            if (instructionsOn) {
              DrivingInstructions.printDrivingInstructions(telemetry);
            }

            telemetry.update();
        } // End of the loop
    }

    // Some methods

    void updateClaw(boolean input){
        switch(grabbingState){
            case OPEN:
                scoringMech.openClaw();
                // If the button is pressed for the first time or the sensor detects a cone, close the claw
                if ((input && !prevClawInput) || scoringMech.getConeStatus()){
                    grabbingState = GrabbingState.ClOSED;
                }
                break;
            case ClOSED:
                scoringMech.closeClaw();
                if (input && !prevClawInput){
                    grabbingState = GrabbingState.WAITING_OPEN;
                }
                break;
            case WAITING_OPEN:
                scoringMech.openClaw();
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
        // Keep the timer at zero until we want it to start ticking, when the claw is closed
        if (!(grabbingState == GrabbingState.ClOSED)) clawActuationTimer.reset();
    }
}
