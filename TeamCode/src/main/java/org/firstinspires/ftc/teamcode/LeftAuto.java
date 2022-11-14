package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.vision.SignalPipeline;

//this autonomous is meant for if you start on the left side of the field
//regular is the red side of the field, -1 is blue side of the field
//@Disabled
@Autonomous
public class LeftAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    SignalPipeline signalPipeline;
    Camera camera;
    Arm arm;
    Lift lift;

    int side = AutoToTele.allianceSide;
    int activeparkzone;

    final double originToWall = 141.0/2.0;
    final double wallDistance = originToWall - 6.5;

    Pose2d startPos = new Pose2d(-35.8*side, -60.6, Math.toRadians(-90*side));
    Pose2d parkPos;
    Pose2d signalPos = new Pose2d(-35.4*side, -34.9);
    Pose2d preloadScoringPos = new Pose2d(-3.4*side, -27.1);
    Trajectory depositPreLoad;
    TrajectorySequence depositCone;
    TrajectorySequence park;
    @Override
    public void runOpMode(){
        // Init
        // Bind stuff to the hardwaremap
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPos);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        camera = new Camera(hardwareMap, signalPipeline);
        FtcDashboard.getInstance().startCameraStream(camera.webcam, 1);
        PhotonCore.enable(); // Loop times go zoom

        ElapsedTime pipelineThrottle = new ElapsedTime();

        // Init loop
        while (!isStarted()&&!isStopRequested()){
            if (gamepad1.b) AutoToTele.allianceSide = 1; //left side
            if (gamepad1.x) AutoToTele.allianceSide = -1; //right side

            // Recompute trajectories every second
            if (pipelineThrottle.milliseconds() > 1000){
                switch(signalPipeline.getParkPos()){
                    case 1:
                        activeparkzone = 1;
                        parkPos = new Pose2d(-58.5*side, -33.0);
                        break;
                    case 2:
                        activeparkzone = 2;
                        parkPos = new Pose2d(-36*side, -26);
                        break;
                    case 3:
                        activeparkzone = 3;
                        parkPos = new Pose2d(-14*side, -27);
                        break;
                }

                // Update trajectories
                depositPreLoad = drive.trajectoryBuilder(startPos)
                        .lineToSplineHeading(new Pose2d(-35.8*side, -60.6,Math.toRadians(180*side)))
                        .addTemporalMarker(1.5, () -> lift.goToHigh())
                        .addTemporalMarker(1, () -> arm.scorePassthrough())
                        .lineToSplineHeading(new Pose2d(-10.5*side, -58.7,Math.toRadians(0*side)))
                        .lineToSplineHeading(preloadScoringPos)
                        .build();

                depositCone = drive.trajectorySequenceBuilder(depositPreLoad.end())
                        .waitSeconds(1)
                        .addTemporalMarker(() -> arm.openClaw())
                        .waitSeconds(1)
                        .build();

                park = drive.trajectorySequenceBuilder(depositCone.end())
                        .addTemporalMarker(() -> arm.grabPassthrough())
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> lift.retract())
                        .splineToSplineHeading(new Pose2d(-12.8*side, -33.9,Math.toRadians(0*side)),Math.toRadians(0*side))
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
        camera.stopStreaming();

        // Start of actual auto instructions
        if (opModeIsActive()){
            // Auto code
            arm.closeClaw();
            sleep(500);
            drive.followTrajectory(depositPreLoad);
            drive.followTrajectorySequence(depositCone);
            drive.followTrajectorySequence(park);

            // Save this stuff at the end to calibrate feild centric automatically
            AutoToTele.endOfAutoPose = drive.getPoseEstimate();
            AutoToTele.endOfAutoHeading = drive.getExternalHeading();
        }
    }
}