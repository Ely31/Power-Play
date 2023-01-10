package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.AutoScoringMech;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.util.AutoConfigUtil;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.workspace.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Config
@Autonomous
public class AsyncAuto2 extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    PivotingCamera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    AutoScoringMech scoringMech;

    AutoConstants autoConstants = new AutoConstants(drive);

    int cycleIndex = 0;

    // For the giant fsm to run everything asynchronously
    enum AutoState{
        GRABBING_PRELOAD,
        SCORING_PRELOAD,
        WAITING_FOR_CONE_GRAB,
        TO_JUNCTION,
        TO_STACK,
        PARKING
    }
    AutoState autoState = AutoState.GRABBING_PRELOAD;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        scoringMech = new AutoScoringMech(hardwareMap);
        camera = new PivotingCamera(hardwareMap, signalPipeline);
        // Juice telemetry speed
        telemetry.setMsTransmissionInterval(100);

        ElapsedTime pipelineThrottle = new ElapsedTime(100000000);
        ElapsedTime actionTimer = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            // Configure the alliance with the gamepad
            if (gamepad1.circle) autoConstants.setSide(1); // Red alliance
            if (gamepad1.cross) autoConstants.setSide(-1); // Blue alliance

            // Recompute trajectories every second
            if (pipelineThrottle.seconds() > 1){
                // Update stuff
                autoConstants.updateParkZoneFromVisionResult(signalPipeline.getParkPos());
                autoConstants.updateParkPos(autoConstants.getParkZone());

                drive.setPoseEstimate(autoConstants.startPos);
                // Display auto configuration to telemetry
                autoConstants.addTelemetry(telemetry);
                telemetry.update();
                pipelineThrottle.reset();
            } // End of throttled section
        }

        waitForStart();
        // Stop the camera because we don't need it and it takes computation
        camera.stopStreaming();
        actionTimer.reset();

        while (opModeIsActive()){
            // One big fsm
            switch (autoState){
                case GRABBING_PRELOAD:
                    // Grab the preload
                    scoringMech.closeClaw();
                    // Once the claw is shut, premove the v4b, then move on to the next state
                    if (actionTimer.milliseconds() > Arm.clawActuationTime){
                        scoringMech.preMoveV4b();
                        // Set the drive on it's next trajectory
                        drive.followTrajectorySequenceAsync(autoConstants.driveToPreloadPos);
                        actionTimer.reset();
                        scoringMech.setRetractedGrabbingPose(0);
                        autoState = AutoState.SCORING_PRELOAD;
                    }
                    break;

                case SCORING_PRELOAD:
                        if (actionTimer.seconds() > 2){
                            scoringMech.scoreAsync(Lift.mediumPos + 0.5);
                        }
                        if (scoringMech.liftIsMostlyDown()){
                            // Send it off again
                            drive.followTrajectorySequenceAsync(autoConstants.toStackFromPreload);
                            actionTimer.reset();
                            // Reset the scoring fsm so it can run again next time
                            scoringMech.resetScoringState();
                            // Start the first cycle
                            if (cycleIndex > autoConstants.getNumCycles()) autoState = AutoState.TO_STACK;
                            // If we don't want to do any cycles, park
                            else autoState = AutoState.PARKING;
                        }
                    break;

                case TO_STACK:
                    scoringMech.grabOffStackAsync(4-cycleIndex, !drive.isBusy());
                    if (!drive.isBusy()){
                        actionTimer.reset();
                        autoState = AutoState.WAITING_FOR_CONE_GRAB;
                    }
                    break;

                case WAITING_FOR_CONE_GRAB:
                    scoringMech.grabOffStackAsync(4-cycleIndex, !drive.isBusy());
                    if (actionTimer.seconds() > autoConstants.stackGrabbingTime){
                        drive.followTrajectorySequenceAsync(autoConstants.toJunction);
                        actionTimer.reset();
                        scoringMech.resetStackGrabbingState();
                        scoringMech.setRetractedGrabbingPose(0);
                        autoState = AutoState.TO_JUNCTION;
                    }
                    break;

                case TO_JUNCTION:
                        if (actionTimer.seconds() > 1.7){
                            scoringMech.scoreAsync(Lift.highPos + 0.5);
                        }
                        if (scoringMech.liftIsMostlyDown()){
                            drive.followTrajectorySequenceAsync(autoConstants.toStack);
                            scoringMech.resetScoringState();
                            actionTimer.reset();
                            // Tell the code we made another cycle
                            cycleIndex ++;
                            // If we've done enough cycles, park
                            if (cycleIndex > autoConstants.getNumCycles()) autoState = AutoState.TO_STACK;
                            else autoState = AutoState.PARKING;
                        }
                    break;

                case PARKING:
                    // Yay, done
                    // Once the bot is parked, stop the OpMode
                    if (!drive.isBusy()){
                        stop();
                    }
                    break;
            }
            // Update all the things
            drive.update();
            scoringMech.updateLift();

            // Save this stuff to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();

            // Show telemetry because there are plenty of bugs it should help me fix
            telemetry.addData("cycle index", cycleIndex);
            telemetry.addData("number of cycles", autoConstants.getNumCycles());
            telemetry.addData("auto state", autoState.name());
            scoringMech.displayAutoMechDebug(telemetry);
            telemetry.update();
        }
    }
}
