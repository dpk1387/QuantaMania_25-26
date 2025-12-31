package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBlueGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)//55 speed here 70 in real robot (roadrunner)
                .build();
        double shootX = -28, shootY = -28, shootYaw = Math.toRadians(-135);
        Pose2d shootPose = new Pose2d(shootX, shootY, shootYaw);
        double newShootX = -16, newShootY = -16;
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(-135));
        Pose2d startPose = new Pose2d(-54, -54, Math.toRadians(-135));
        Pose2d classifierPose = new Pose2d(7.5, -64, Math.toRadians(-120));
        double readyX = 4, readyY = -30, readyYaw = Math.toRadians(-45);
        Pose2d readyPose = new Pose2d(readyX, readyY, readyYaw);

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // preloads
                .strafeTo(new Vector2d(newShootX, newShootY))

                .waitSeconds(2) //to shoot

//                 middle row of artifacts
                .setTangent(Math.toRadians(-15))
                .splineToSplineHeading(new Pose2d(2, -38,  Math.toRadians(-45)), Math.toRadians(-30)) //go into
                .splineToLinearHeading(new Pose2d(4, -62,  Math.toRadians(-110)), Math.toRadians(-95)) //go into
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(readyX-2, readyY, shootYaw), Math.toRadians(135)) //go into
                .splineToLinearHeading(shootPose, Math.toRadians(200)) //go into
                .waitSeconds(2)

                // classifier artifacts (1)
                .splineToLinearHeading(new Pose2d(readyX, readyY, Math.toRadians(-120)), Math.toRadians(-55)) //-95 //go into
                .splineToSplineHeading(classifierPose, Math.toRadians(-60))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(60)) //-95
                .splineToSplineHeading(new Pose2d(readyX-4, readyY, shootYaw), Math.toRadians(135))
                .splineToLinearHeading(shootPose, Math.toRadians(135)) //200 //go into
                .waitSeconds(2)

                // classifier artifacts (2)
                .setTangent(Math.toRadians(-15))
                .splineToSplineHeading(new Pose2d(readyX, readyY, Math.toRadians(-120)), Math.toRadians(-72))
                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //go into
                .waitSeconds(1.5)

                .splineToSplineHeading(new Pose2d(readyX-4, readyY, shootYaw), Math.toRadians(135))
                .splineToLinearHeading(shootPose, Math.toRadians(200)) //go into
                .waitSeconds(2)

                // first line of artifacts
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-24, -37,  Math.toRadians(-45)), Math.toRadians(-45)) //go into
                .splineToLinearHeading(new Pose2d(-8, -60,  Math.toRadians(-95)), Math.toRadians(-90)) //go into
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(-225)) //go into
                .waitSeconds(2)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
