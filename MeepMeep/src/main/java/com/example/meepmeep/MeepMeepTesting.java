package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.DisplacementMarker;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.MeepMeep.Background.*;

import static java.lang.Math.*;

public class MeepMeepTesting {
    static double botWidth = 12, botHeight = 15;
    static double tileSize_Inches = 24;
    static double hubRadius = 9;
    static double offset = 1;
    static double wait = .45;
    static double sideModifier = 1; // -1 for blue alliance


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), botWidth)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-3d * tileSize_Inches + botHeight/2d, -.5d * tileSize_Inches + botHeight / 2d))
                                .waitSeconds(120)
                                .build()
                );

        myBot.setDimensions(botWidth, botHeight);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }


    void CycleAuto() {
        /*(new Pose2d(.5d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d, toRadians(90)))
                .lineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                // starts cycle #1
                .splineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset), toRadians(90))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                // starts cycle #2
                .splineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset), toRadians(90))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                // starts cycle #3
                .splineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset), toRadians(90))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                // starts cycle #4
                .splineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset), toRadians(90))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                // starts cycle #5
                .splineTo(new Vector2d(-.5d * tileSize_Inches, -tileSize_Inches - hubRadius - botHeight / 2d - offset), toRadians(90))
                .waitSeconds(wait)
                .setReversed(true)
                .splineTo(new Vector2d(2d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d + offset), toRadians(0))
                .setReversed(false)
                .build()*/
    }

    void DuckAuto() {
        /*(new Pose2d(-1.5d * tileSize_Inches, -3d * tileSize_Inches + botHeight / 2d, toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-tileSize_Inches - hubRadius / 2d - offset, -1.75 * tileSize_Inches, toRadians(0)))
                                .strafeLeft(.75d * tileSize_Inches)
                                .waitSeconds(wait)
                                .strafeRight(.75d * tileSize_Inches)
                                .lineTo(new Vector2d(-2d * tileSize_Inches, -3 * tileSize_Inches + botHeight / 2d + offset))
                                .waitSeconds(3)
                                .lineToSplineHeading(new Pose2d(-1.5d * tileSize_Inches, -3 * tileSize_Inches + botHeight + offset, toRadians(75)))
                                .lineTo(new Vector2d(-1.5d * tileSize_Inches, -3 * tileSize_Inches + botHeight / 2d + 6d * offset))
                                .lineTo(new Vector2d(-2.25d * tileSize_Inches, -3 * tileSize_Inches + botHeight / 2d + 6d * offset))
                                .lineToSplineHeading(new Pose2d(-2.25d * tileSize_Inches, -3 * tileSize_Inches + botHeight + offset, toRadians(90)))
                                //.turn(toRadians(15))
                                .splineTo(new Vector2d(-3d * tileSize_Inches + botHeight / 2d + 3d * offset, -tileSize_Inches), toRadians(90))
                                .lineToSplineHeading(new Pose2d(-tileSize_Inches - hubRadius / 2d - offset, -tileSize_Inches, toRadians(0)))
                                .waitSeconds(wait)
                                .lineTo(new Vector2d(-3d * tileSize_Inches + botHeight / 2d + 6d * offset, -tileSize_Inches))
                                .strafeRight(.5d * tileSize_Inches)
                                .build()*/
    }

    void GoalAuto() {
        /*(new Pose2d(-3d * tileSize_Inches + botHeight / 2d, -tileSize_Inches))
                                .waitSeconds(3)
                                .splineTo(new Vector2d(.5 * tileSize_Inches, -2d * tileSize_Inches + botHeight / 2d), toRadians(-90))
                                .waitSeconds(3)
                                .lineToSplineHeading(new Pose2d(-.75d * tileSize_Inches, -1.5d * tileSize_Inches))
                                .waitSeconds(3)
                                .back(.25d * tileSize_Inches)
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(-2d * tileSize_Inches + botHeight, -2d * tileSize_Inches, toRadians(180)))
                                .waitSeconds(3)
                                .forward(.25d * tileSize_Inches)
                                .waitSeconds(2)
                                .lineToSplineHeading(new Pose2d(-botHeight / 2d, -2.5d * tileSize_Inches))
                                .waitSeconds(3)
                                .lineTo(new Vector2d(.5d * tileSize_Inches, -1.5d * tileSize_Inches))
                                .build()*/
    }
}