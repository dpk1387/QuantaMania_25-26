package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        double shootX = -24, shootY = 24;
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, 48, Math.toRadians(135)))
                .strafeTo(new Vector2d(shootX, shootY))
                .waitSeconds(3) //to shoot
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-11, 28,  Math.toRadians(90)), Math.toRadians(45)) //go into
                //.splineTo(new Vector2d(-11, 24), Math.toRadians(90))
                .strafeTo(new Vector2d(-11, 52))
                .strafeTo(new Vector2d(-6, 46))
                .strafeTo(new Vector2d(-6, 52))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(11, 28,  Math.toRadians(90)), Math.toRadians(20))
                .strafeTo(new Vector2d(11, 56))
                .strafeTo(new Vector2d(6, 48))
                .strafeTo(new Vector2d(-6, 52))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(33, 28,  Math.toRadians(90)), Math.toRadians(15))
                .strafeTo(new Vector2d(33, 56))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(135)), Math.toRadians(225))
                .waitSeconds(3) //to shoot
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}