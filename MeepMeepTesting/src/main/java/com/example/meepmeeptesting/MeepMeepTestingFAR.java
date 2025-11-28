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
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        double shootX = -6, shootY = 13, shootYaw = 140;//135;
        Pose2d shootPose = new Pose2d(shootX, shootY,  Math.toRadians(shootYaw));
        double inX = 54, inY = 58;
        double turnX = inX - 8, turnY = 11;


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 11, Math.toRadians(180)))
                .splineToLinearHeading(shootPose, Math.toRadians(180))
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(0))
                //go to intake the 3rd set of balls
                .splineToLinearHeading(new Pose2d(turnX, turnY, Math.toRadians(90)),Math.toRadians(0))
                //strafe forwards to intake
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(inX, inY, Math.toRadians(90)),Math.toRadians(90))
                .turn(Math.toRadians(-45))
                //.turn(Math.toRadians(45))
                //.strafeTo(new Vector2d(inX + 2, inY+4))
                //go back
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(turnX, turnY, Math.toRadians(125)),Math.toRadians(-135))
                .setTangent(Math.toRadians(180))
                //go to shoot
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                .setTangent(Math.toRadians(0))
                //go to intake the 3rd set of balls
                .splineToLinearHeading(new Pose2d(turnX, turnY, Math.toRadians(90)),Math.toRadians(0))
                //strafe forwards to intake
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(inX, inY, Math.toRadians(90)),Math.toRadians(90))
                .turn(Math.toRadians(-45))
                //.turn(Math.toRadians(45))
                //.strafeTo(new Vector2d(inX + 2, inY+4))
                //go back
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(turnX, turnY, Math.toRadians(125)),Math.toRadians(-135))
                .setTangent(Math.toRadians(180))
                //go to shoot
                .splineToLinearHeading(new Pose2d(shootX, shootY,  Math.toRadians(shootYaw)), Math.toRadians(180)) //go into
                .waitSeconds(3) //to shoot

                /*
                //----------------------------------------------------------------------------------
                // We want to travel CLOCKWISE, so start tangent is "down" (-90° / 270°)
                .setTangent(Math.toRadians(-90))

                // 1) Right -> Bottom (bottom point has tangent = left / 180°)
                .splineToConstantHeading(
                        new Vector2d(cx, cy - R),           // bottom
                        Math.toRadians(180)                 // path tangent: left
                )

                // 2) Bottom -> Left (left point has tangent = up / 90°)
                .splineToConstantHeading(
                        new Vector2d(cx - R, cy),           // left
                        Math.toRadians(90)                  // path tangent: up
                )

                // 3) Left -> Top (top point has tangent = right / 0°)
                .splineToConstantHeading(
                        new Vector2d(cx, cy + R),           // top
                        Math.toRadians(0)                   // path tangent: right
                )

                // 4) Top -> Right (back to start, tangent = down / -90°)
                .splineToConstantHeading(
                        new Vector2d(cx + R, cy),           // right (start point)
                        Math.toRadians(-90)                 // path tangent: down
                )

                 */

                //----------------------------------------------------------------------------------
                /*
                //circling, counter clockwise
                .splineToConstantHeading(
                        new Vector2d(cx, cy + R),              // top
                        Math.toRadians(180)                    // path tangent (left)
                )
                // quarter 2: 90° → 180°
                .splineToConstantHeading(
                        new Vector2d(cx - R, cy),              // left
                        Math.toRadians(270)                    // path tangent (down)
                )
                // quarter 3: 180° → 270°
                .splineToConstantHeading(
                        new Vector2d(cx, cy - R),              // bottom
                        Math.toRadians(0)                      // path tangent (right)
                )
                // quarter 4: 270° → 360° (back to start)
                .splineToConstantHeading(
                        new Vector2d(cx + R, cy),              // right (start point)
                        Math.toRadians(90)                     // path tangent (up)
                )
                */
                //----------------------------------------------------------------------------------
                //.setTangent(Math.toRadians(170))
                //.splineToLinearHeading(new Pose2d(54, 57,  Math.toRadians(90)), Math.toRadians(180)) //go into.
                //.setTangent(Math.toRadians(-90))
                //.splineToLinearHeading(new Pose2d(58, 57,  Math.toRadians(90)), Math.toRadians(90)) //go into.
                /*
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
                 */
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}