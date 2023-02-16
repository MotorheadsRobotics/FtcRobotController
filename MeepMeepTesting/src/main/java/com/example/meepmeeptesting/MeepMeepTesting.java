package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.entity.BotEntity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(11.75, 17)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -65.75, Math.toRadians(0)))
                                .setReversed(true)
                                .addDisplacementMarker(() -> {
                                    // Set Lift High Goal
                                })
                                .strafeLeft(47.75)
                                .splineToSplineHeading(new Pose2d(-30.8,-6.8,Math.toRadians(225)), Math.toRadians(45))
                                .lineTo(new Vector2d(-26.8,-2.8))
                                .addDisplacementMarker(() -> {
                                    // Down
                                })
                                .addDisplacementMarker(() -> {
                                    // Drop
                                })
                                .addDisplacementMarker(() -> {
                                    // Up
                                })

                                .setReversed(false)
                                .addTemporalMarker(() -> {
                                    // Lift to cone 5
                                })
                                .splineTo(new Vector2d(-63.5,-12), Math.toRadians(180))
                                .addDisplacementMarker(() -> {
                                    // Claw Close
                                })

                                .setReversed(true)
                                .addTemporalMarker(() -> {
                                    // Lift to high goal
                                })
                                .splineTo(new Vector2d(-26.8,-2.8), Math.toRadians(45))
                                .addDisplacementMarker(() -> {
                                    // Claw Close
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}