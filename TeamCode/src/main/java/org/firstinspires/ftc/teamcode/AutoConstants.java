package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AutoConfigUtil;

public class AutoConstants {
    AutoConfigUtil autoConfigUtil = new AutoConfigUtil();
    SampleMecanumDrive drive;

    int side = autoConfigUtil.getAllianceSide();

    double grabApproachVelo = 10;

    // Stored pose info to be accesed in teleop
    public static Pose2d endOfAutoPose = new Pose2d(0,0, Math.toRadians(-90));

    // Constructor
    public AutoConstants(SampleMecanumDrive drive){
        this.drive = drive;
    }

    // Pose2d's
    public Pose2d startPos = new Pose2d(-35.8, -63*side, Math.toRadians(-90*side));
    public Pose2d preloadScoringPos = new Pose2d(-37.5, -6.5*side, Math.toRadians(131*side));

    // Set parkPos to a default to avoid null issues
    public Pose2d parkPos = new Pose2d(-57, -12*side, Math.toRadians(180*side));

    public Pose2d[] parkPositions = {
            // Pos 1
            new Pose2d(-57, -12*side, Math.toRadians(180*side)),
            // Pos 2
            new Pose2d(-36, -12*side, Math.toRadians(180*side)),
            // Pos 3
            new Pose2d(-11, -12*side, Math.toRadians(180*side))
    };
    public void updateParkPos(int posIndex){
        // Grab the correct pos from the array and set parkPos to it
        parkPos = parkPositions[posIndex-1];
    }


    // Trajectories
    public TrajectorySequence driveToPreloadPos = drive.trajectorySequenceBuilder(startPos)
            .lineToSplineHeading(preloadScoringPos)
            .build();

    public TrajectorySequence toStackFromPreload = drive.trajectorySequenceBuilder(driveToPreloadPos.end())
            .setTangent(Math.toRadians(-120*side))
            .splineToSplineHeading(new Pose2d(-58,-12.2*side, Math.toRadians(180*side)), Math.toRadians(180*side))
            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
            .lineTo(new Vector2d(-64, -12.2*side))
            .build();

    public TrajectorySequence toJunction = drive.trajectorySequenceBuilder(toStackFromPreload.end())
            .lineTo(new Vector2d(-35, -12*side))
            .splineToSplineHeading(new Pose2d(-21, -12*side, Math.toRadians(150*side)), Math.toRadians(0*side))
            .build();

    public TrajectorySequence toStack = drive.trajectorySequenceBuilder(toJunction.end())
            .setTangent(Math.toRadians(180*side))
            .splineToSplineHeading(new Pose2d(-58,-12.1*side, Math.toRadians(180*side)), Math.toRadians(180*side))
            .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(grabApproachVelo, DriveConstants.MAX_ANG_VEL, 13.2))
            .lineTo(new Vector2d(-63.5, -12.1*side))
            .build();

    public TrajectorySequence toCloseHighJunction = drive.trajectorySequenceBuilder(toStack.end())
            .lineToSplineHeading(new Pose2d(-41.5, -12*side, Math.toRadians(-144)))
            .build();

    public TrajectorySequence park = drive.trajectorySequenceBuilder(toCloseHighJunction.end())
            .lineToSplineHeading(parkPos)
            .build();
}
