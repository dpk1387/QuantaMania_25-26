package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
/*
* THis version for autonomous far stating point
* */
public class MeepMeepTestingFAR {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        double shootX = -11, shootY = 11, shootYaw = 135;
        double intakeY = 30, intakeYaw = 90;
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 11, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(60, 30,  Math.toRadians(90)), Math.toRadians(45)) //go into
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 30))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(60, 11,  Math.toRadians(90)), Math.toRadians(0)) //go into
                .splineToLinearHeading(new Pose2d(35, intakeY, Math.toRadians(90)),Math.toRadians(45))
                //.strafeTo(new Vector2d(60, 60))
                //.strafeTo(new Vector2d(60, 11))
                .strafeTo(new Vector2d(35, 52))
                .strafeTo(new Vector2d(35, intakeY))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(60, 11,  Math.toRadians(90)), Math.toRadians(0)) //go into
                .splineToLinearHeading(new Pose2d(12, intakeY, Math.toRadians(90)), Math.toRadians(80))
                //.strafeTo(new Vector2d(60, 60))
                //.strafeTo(new Vector2d(60, 11))
                .strafeTo(new Vector2d(12, 52))
                .strafeTo(new Vector2d(12, intakeY))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                //.splineTo(new Vector2d(-11, 24), Math.toRadians(90))
                //.strafeTo(new Vector2d(-11, 52))
                //.setTangent(Math.toRadians(225))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}