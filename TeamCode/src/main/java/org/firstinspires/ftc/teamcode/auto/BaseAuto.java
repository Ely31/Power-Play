package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class BaseAuto extends LinearOpMode {
    // Pre init
    SampleMecanumDrive drive;
    Pose2d startpos = new Pose2d(0,0,Math.toRadians(0));

    TrajectorySequence traj;
    @Override
    public void runOpMode(){
        // Init
        PhotonCore.enable();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startpos);

        traj = drive.trajectorySequenceBuilder(startpos)
                .lineToSplineHeading(new Pose2d(20,20,Math.toRadians(180)))
                .build();

        waitForStart();
        if (opModeIsActive()){
            // Auto code
            drive.followTrajectorySequence(traj);
        }
    }
}