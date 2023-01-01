package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.AutoScoringMech;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.PivotingCamera;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.vision.workspace.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Config
@Autonomous
public class AsyncAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    PivotingCamera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    AutoScoringMech scoringMech;

    int side = 1;
    String sidename;

    Pose2d startPos;
    Pose2d parkPos;
    Pose2d parkPos1;
    Pose2d parkPos2;
    Pose2d parkPos3;
    Pose2d preloadScoringPos;
    TrajectorySequence driveToPreloadPos;
    TrajectorySequence toStackFromPreload;
    double grabApproachVelo = 5;
    TrajectorySequence toJunction;
    TrajectorySequence park;

    enum AutoState{
        GRABBING_PRELOAD,
        SCORING_PRELOAD,
        TO_STACK_FROM_PRELOAD,
        WAITING_FOR_CONE_GRAB,
        TO_JUNCTION,
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

        ElapsedTime pipelineThrottle = new ElapsedTime(100000000);
        ElapsedTime actionTimer = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            if (gamepad1.circle) side = 1; // Red alliance
            if (gamepad1.cross) side = -1; // Blue alliance
            AutoToTele.allianceSide = side;
            // Is doing an if else statement on one line bad?
            if (side == 1) sidename = "Red Terminal"; else sidename = "Blue Terminal";

            // Recompute trajectories every second
            if (pipelineThrottle.milliseconds() > 1000){

                startPos = new Pose2d(-35.8, -63*side, Math.toRadians(-90*side));
                drive.setPoseEstimate(startPos);

                parkPos1 = new Pose2d(-58.5, -13*side, Math.toRadians(180*side));
                parkPos2 = new Pose2d(-36, -13*side, Math.toRadians(180*side));
                parkPos3 = new Pose2d(-11, -13*side, Math.toRadians(180*side));

                switch(signalPipeline.getParkPos()){
                    case 1:
                        // Switch 1 and 3 if we're on blue terminal
                        if (side == 1){
                            parkPos = parkPos1;
                        } else {
                            parkPos = parkPos3;
                        }
                        break;
                    case 2:
                        // We don't have to change the middle positon ever
                        parkPos = parkPos2;
                        break;
                    case 3:
                        // Switch 3 and 1 if we're on blue terminal
                        if (side == 1) {
                            parkPos = parkPos3;
                        } else {
                            parkPos = parkPos1;
                        }
                        break;
                }

                // Update trajectories
                preloadScoringPos = new Pose2d(-37, -7*side, Math.toRadians(131*side));

                driveToPreloadPos = drive.trajectorySequenceBuilder(startPos)
                        .lineToSplineHeading(preloadScoringPos)
                        .build();

                toStackFromPreload = drive.trajectorySequenceBuilder(driveToPreloadPos.end())
                        .setTangent(Math.toRadians(-120*side))
                        .splineToSplineHeading(new Pose2d(-55,-12.5*side, Math.toRadians(180*side)), Math.toRadians(180*side))
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
                        .lineTo(new Vector2d(-64, -12.5*side))
                        .build();

                toJunction = drive.trajectorySequenceBuilder(toStackFromPreload.end())
                        .lineTo(new Vector2d(-30, -12*side))
                        .splineToSplineHeading(new Pose2d(-21, -14*side, Math.toRadians(150*side)), Math.toRadians(-30*side))
                        .build();

                park = drive.trajectorySequenceBuilder(toJunction.end())
                        .lineToSplineHeading(parkPos)
                        .build();

                // Display what park zone the robot is planning on going to and other things in telemetry
                telemetry.addLine(sidename);
                telemetry.addData("alliance", AutoToTele.allianceSide);
                telemetry.addData("park zone", signalPipeline.getParkPos());
                telemetry.addLine("CHECK THE AUTO, REMEMBER NANO FINALS 3!");
                telemetry.update();
                pipelineThrottle.reset();
            } // End of throttled section
        }

        waitForStart();
        camera.stopStreaming();
        // Start of actual auto instructions
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
                        drive.followTrajectorySequenceAsync(driveToPreloadPos);
                        actionTimer.reset();
                        autoState = AutoState.SCORING_PRELOAD;
                    }
                    break;

                case SCORING_PRELOAD:
                        if (actionTimer.seconds() > 2){
                            scoringMech.scoreAsync(Lift.mediumPos);
                        }
                        if (scoringMech.getScoringState() == AutoScoringMech.ScoringState.DONE){
                            // Send it off again
                            drive.followTrajectorySequenceAsync(toStackFromPreload);
                            actionTimer.reset();
                            // Reset the scoring fsm so it can run again next time
                            scoringMech.resetScoringState();
                            autoState = AutoState.TO_STACK_FROM_PRELOAD;
                        }
                    break;

                case TO_STACK_FROM_PRELOAD:
                    scoringMech.grabOffStackAsync(4, !drive.isBusy());
                    if (!drive.isBusy()){
                        actionTimer.reset();
                        autoState = AutoState.WAITING_FOR_CONE_GRAB;
                    }
                    break;

                case WAITING_FOR_CONE_GRAB:
                    scoringMech.grabOffStackAsync(4, !drive.isBusy());
                    if (actionTimer.seconds() > 1){
                        drive.followTrajectorySequenceAsync(toJunction);
                        actionTimer.reset();
                        scoringMech.resetStackGrabbingState();
                        autoState = AutoState.TO_JUNCTION;
                    }
                    break;

                case TO_JUNCTION:
                        if (actionTimer.seconds() > 3){
                            scoringMech.scoreAsync(Lift.highPos);
                        }
                        if (scoringMech.getScoringState() == AutoScoringMech.ScoringState.DONE){
                            drive.followTrajectorySequenceAsync(park);
                            scoringMech.resetScoringState();
                            actionTimer.reset();
                            autoState = AutoState.PARKING;
                        }
                    break;

                case PARKING:
                    // Yay, done
                    break;
            }

            // Update all the things
            drive.update();

            // Save this stuff to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();

            // Show telemetry because there are plenty of bugs it should help me fix
            scoringMech.displayDebug(telemetry);
            telemetry.update();
        }
    }
}

//luke was here
// And Ely