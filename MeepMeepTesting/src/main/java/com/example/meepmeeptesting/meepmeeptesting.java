package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

    public class meepmeeptesting {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(800);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)
                    .build();

            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-14.5, -64, 0))
                    .setTangent(Math.toRadians(90))
                    .lineToY(-56)
                    .setTangent(0)
                    .lineToXLinearHeading(-55,Math.toRadians(45.6557))
                    .setTangent(90)
                    .splineToLinearHeading(new Pose2d(-55.5,-48,Math.toRadians(98)),90)
                    .setTangent(90)
                    .splineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)),90)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-54,-46,Math.toRadians(70)),0)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-55,-56, Math.toRadians(45)),0)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-46,-29,Math.toRadians(-180)),0)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-55,-56, Math.toRadians(45)),0)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-29,-10,Math.toRadians(0)),0)
                    .build());

            meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
