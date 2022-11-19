package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoToTele;
import org.firstinspires.ftc.teamcode.util.Utility;
import org.firstinspires.ftc.teamcode.vision.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
@Config
@Autonomous
public class LeftAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Camera camera;
    SignalPipeline signalPipeline = new SignalPipeline();
    Arm arm;
    Lift lift;

    int side = 1;
    public static int activeparkzone = 1;

    Pose2d startPos = new Pose2d(-35.8, -60.6* side, Math.toRadians(-90*side));
    Pose2d parkPos;
    Pose2d preloadScoringPos = new Pose2d(-15*side, -50, Math.toRadians(-112*side));
    TrajectorySequence driveToScoringPos;
    TrajectorySequence park;

    enum ScoringState{
        EXTENDING,
        V4BOUT,
        WAIT,
        V4BIN,
        WAIT2,
        RETRACTING,
        DONE
    }
    ScoringState currentScoringState = ScoringState.EXTENDING;

    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        camera = new Camera(hardwareMap, signalPipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 3);

        ElapsedTime pipelineThrottle = new ElapsedTime();
        ElapsedTime scoringWait = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            if (gamepad1.circle) side = 1; // Red alliance
            if (gamepad1.cross) side = -1; // Blue alliance
            AutoToTele.allianceSide = side;

            // Recompute trajectories every second
            if (pipelineThrottle.milliseconds() > 1000){

                startPos = new Pose2d(-35.8, -60.6*side, Math.toRadians(-90*side));
                drive.setPoseEstimate(startPos);

                // Should be switching pipeline.getParkPos, but just testing now
                switch(signalPipeline.getParkPos()){
                    case 1:
                        activeparkzone = 1;
                        parkPos = new Pose2d(-58.5, -32*side, Math.toRadians(-90*side));
                        break;
                    case 2:
                        activeparkzone = 2;
                        parkPos = new Pose2d(-36, -32*side, Math.toRadians(-90*side));
                        break;
                    case 3:
                        activeparkzone = 3;
                        parkPos = new Pose2d(-11, -32*side, Math.toRadians(-90*side));
                        break;
                }

                // Update trajectories
                preloadScoringPos = new Pose2d(-10, -32.5*side, Math.toRadians(-135*side));

                driveToScoringPos = drive.trajectorySequenceBuilder(startPos)
                        .back(2.5)
                        .lineToSplineHeading(new Pose2d(-10, -56*side,Math.toRadians(-90*side)))
                        .lineToSplineHeading(preloadScoringPos)
                        .build();


                park = drive.trajectorySequenceBuilder(driveToScoringPos.end())
                        .lineToSplineHeading(parkPos)
                        .build();

                // Display what park zone the robot is planning on going to and other things in telemetry
                telemetry.addData("alliance", AutoToTele.allianceSide);
                telemetry.addData("park zone", activeparkzone);
                telemetry.addLine("CHECK THE AUTO, REMEMBER NANO FINALS 3!");
                telemetry.update();
                pipelineThrottle.reset();
            } // End of throttled section
        }

        waitForStart();
        //camera.stopStreaming();

        // Start of actual auto instructions
        if (opModeIsActive()){
            // Auto code
            arm.closeClaw();
            sleep(500);
            //lift.setHeight(1); // So the cone doesn't drag on the ground
            drive.followTrajectorySequence(driveToScoringPos);

            // While it isn't finished scoring, run an FSM
            while (!(currentScoringState == ScoringState.DONE)){
                switch (currentScoringState){
                    case EXTENDING:
                        lift.goToHigh();
                        // Move on if the lift is all the way up
                        if (Utility.withinErrorOfValue(lift.getHeight(), Lift.highPos, 0.5)) {
                            arm.scorePassthrough();
                            scoringWait.reset();
                            currentScoringState = ScoringState.WAIT;
                        }
                        break;
                    case WAIT:
                        if (scoringWait.seconds() > 2){
                            arm.openClaw();
                            scoringWait.reset();
                            currentScoringState = ScoringState.WAIT2;
                        }
                        break;
                    case WAIT2:
                        if (scoringWait.seconds() > 2){
                            arm.grabPassthrough();
                            currentScoringState = ScoringState.RETRACTING;
                        }
                        break;
                    case RETRACTING:
                        lift.retract();
                        // Move on if the lift is all the way up
                        if (Utility.withinErrorOfValue(lift.getHeight(), Lift.retractedPos, 0.5)) {
                            currentScoringState = ScoringState.DONE;
                        }
                        break;
                }
                // Always update the lift, no matter what state of scoring it's in
                lift.update();
            }

            drive.followTrajectorySequence(park);

            // Save this stuff at the end to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
        }
    }
}

//luke was here