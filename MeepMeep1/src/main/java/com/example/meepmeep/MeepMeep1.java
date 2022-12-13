package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeep1 {

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");

        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800, 30);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-8.8, -14.6,Math.toRadians(-135)))
                                .setTangent(Math.toRadians(180))
                                .splineToSplineHeading(new Pose2d(-40,-12, Math.toRadians(180)), Math.toRadians(180))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, Math.toRadians(120), 13.2))
                                .lineTo(new Vector2d(-60, -12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.9f)
                .addEntity(myBot)
                .start();
    }
}