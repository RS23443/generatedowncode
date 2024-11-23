package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, 66, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-5,34), Math.toRadians(90))
                        .splineTo(new Vector2d(-5,30), Math.toRadians(90))
                        .splineTo(new Vector2d(-48,48), Math.toRadians(270))
                        .turnTo(Math.toRadians(270-21.6))
                        .turnTo(Math.toRadians(270))
                        .turnTo(Math.toRadians(270-40))
                        .turnTo(Math.toRadians(270))
                        .splineTo(new Vector2d(-35,60), Math.toRadians(270))
                        .splineTo(new Vector2d(-10,34), Math.toRadians(90))
                .splineTo(new Vector2d(-35,60), Math.toRadians(270))
                .splineTo(new Vector2d(-12,34), Math.toRadians(90))
                .splineTo(new Vector2d(-35,60), Math.toRadians(270))
                .splineTo(new Vector2d(-9,34), Math.toRadians(90))
                .splineTo(new Vector2d(-50,60), Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}