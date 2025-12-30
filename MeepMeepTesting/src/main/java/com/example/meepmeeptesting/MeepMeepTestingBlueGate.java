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
        double shootX = -28, shootY = -28;
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(-135));
        double newShootX = -16, newShootY = -16;
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(-135));
        Pose2d startPose = new Pose2d(-54, -54, Math.toRadians(-135));
        Pose2d classifierPose = new Pose2d(7.5, -64, Math.toRadians(-120));
        Pose2d readyPose = new Pose2d(4, -30, Math.toRadians(-45));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // preloads
                .strafeTo(new Vector2d(newShootX, newShootY))

                .waitSeconds(2) //to shoot
                //.turn(-Math.toRadians(100))
                //.strafeTo(new Vector2d(-20, 40))

                // middle row of artifacts
                .setTangent(Math.toRadians(5))
                //.splineToLinearHeading(readyPose, Math.toRadians(-30)) //go into
                .splineToLinearHeading(new Pose2d(12, -58, Math.toRadians(-110)), Math.toRadians(-95)) //go into
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(newShootPose, Math.toRadians(160)) //145 //go into
                .waitSeconds(2)

                // classifier artifacts (1)
                .setTangent(Math.toRadians(5))
                .splineToLinearHeading(classifierPose, Math.toRadians(-80))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(newShootPose, Math.toRadians(160))
                .waitSeconds(2)

                // classifier artifacts (2)
                .setTangent(Math.toRadians(5))
                .splineToLinearHeading(classifierPose, Math.toRadians(-80))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(newShootPose, Math.toRadians(160))
                .waitSeconds(2)

                // first line of artifacts
                .setTangent(Math.toRadians(-80))
                //.splineToLinearHeading(new Pose2d(-24, -38,  Math.toRadians(-45)), Math.toRadians(-45)) //go into
                .splineToLinearHeading(new Pose2d(-11, -58,  Math.toRadians(-95)), Math.toRadians(-90)) //go into
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