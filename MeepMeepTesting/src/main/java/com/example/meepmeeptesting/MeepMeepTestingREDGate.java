package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingREDGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)//55 speed here 70 in real robot (roadrunner)
                .build();
        double shootX = -28, shootY = 28;
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(135));

        //longer shot
        //shorter movement to classifier
        double newShootX = -16, newShootY = 16; // -22, 22
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(135));

        Pose2d startPose = new Pose2d(-54, 54, Math.toRadians(135));
        Pose2d classifierPose = new Pose2d(7.5, 62, Math.toRadians(120));

        //not necessary
        Pose2d readyPose = new Pose2d(4, -30, Math.toRadians(-45));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                //SHOOT PRELOADS
                .strafeTo(new Vector2d(newShootX, newShootY))
                .waitSeconds(2)

                //.turn(-Math.toRadians(100))
                //.strafeTo(new Vector2d(-20, 40))

                // MIDDLE ROW OF ARTIFACTS
                .setTangent(Math.toRadians(5))
                //.splineToLinearHeading(readyPose, Math.toRadians(-30))
                //chatgpt suggestion:
//                .splineToLinearHeading(new Pose2d (11, 38, Math.toRadians(90)), Math.toRadians(100))

                //pick up
                //.splineToSplineHeading(new Pose2d(10, 56, Math.toRadians(110)), Math.toRadians(110)) //_, _,_, 95//
                .splineToSplineHeading(new Pose2d(10+2, 56-7, Math.toRadians(90)), Math.toRadians(80)) // 80

                //go back to shoot
                //.setTangent(Math.toRadians(-100))
                .splineToLinearHeading(newShootPose, Math.toRadians(-160))
                .waitSeconds(2)

                // classifier artifacts (1)
                .setTangent(Math.toRadians(2)) //5
                .splineToLinearHeading(classifierPose, Math.toRadians(80))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(-100))
                .splineToLinearHeading(newShootPose, Math.toRadians(-160))
                .waitSeconds(2)

                // classifier artifacts (2)
                .setTangent(Math.toRadians(2)) //5
                .splineToLinearHeading(classifierPose, Math.toRadians(80))
                .waitSeconds(1.5)

                .setTangent(Math.toRadians(-100))
                .splineToLinearHeading(newShootPose, Math.toRadians(-160))
                .waitSeconds(2)

                // first line of artifacts

                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-24, 38,  Math.toRadians(45)), Math.toRadians(45))
                //.splineToLinearHeading(new Pose2d(-11, 54,  Math.toRadians(90)), Math.toRadians(80))
                .splineToSplineHeading(new Pose2d(-11, 54-5,  Math.toRadians(90)), Math.toRadians(90))

                        //.setTangent(Math.toRadians(-90))
                .splineToLinearHeading(shootPose, Math.toRadians(225)) //go into
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