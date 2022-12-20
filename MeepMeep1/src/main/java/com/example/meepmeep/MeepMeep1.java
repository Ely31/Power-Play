package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeep1 {

    public static void main(String[] args) {

        System.setProperty("sun.java2d.opengl", "true");

        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800, 30);

        RoadRunnerBotEntity Bot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 30, Math.toRadians(120), Math.toRadians(120), 13.2)
                .setDimensions(15,15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-8.8, -39.5,Math.toRadians(-126)))
                                .lineToSplineHeading(new Pose2d(-11.1, -20, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-55,-12), Math.toRadians(180))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, Math.toRadians(120), 13.2))
                                .lineTo(new Vector2d(-60, -12))
                                .build()
                );

        RoadRunnerBotEntity Bot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 30, Math.toRadians(120), Math.toRadians(120), 13.2)
                .setDimensions(15,15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -12,Math.toRadians(180)))
                                .lineTo(new Vector2d(-30, -12))
                                .splineTo(new Vector2d(-12, -14), Math.toRadians(-30))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.9f)
                .addEntity(Bot1)
                .addEntity(Bot2)
                .start();
    }
}