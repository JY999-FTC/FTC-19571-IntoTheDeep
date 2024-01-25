package spudnik.team24288.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    final static String SEQUENCE = "BlueBottom";
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;

        switch (SEQUENCE) {
            case "BlueBottom":
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37.24, 67.47, Math.toRadians(269.24)))
                                .lineToConstantHeading(new Vector2d(-36.59, 35.51))
                                .lineToConstantHeading(new Vector2d(16.52, 35.51))
                                .lineToSplineHeading(new Pose2d(44.37, 35.51, Math.toRadians(180.00)))
                                .build()
                    );
                break;
            case "RedBottom":
                myBot = new DefaultBotBuilder(meepMeep)
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35.46, -68.29, Math.toRadians(448.94)))
                                .splineTo(new Vector2d(-36.05, -24.72), Math.toRadians(90.77))
                                .splineTo(new Vector2d(24.00, -11.00), Math.toRadians(376.96))
                                .lineToSplineHeading(new Pose2d(53.05, -36.05, Math.toRadians(181.45)))
                                .build()
                );
                break;
            default:
                myBot = new DefaultBotBuilder(meepMeep)
                        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                        .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                        .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .forward(30)
                                        .turn(Math.toRadians(90))
                                        .build()
                        );
                break;
        }



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}