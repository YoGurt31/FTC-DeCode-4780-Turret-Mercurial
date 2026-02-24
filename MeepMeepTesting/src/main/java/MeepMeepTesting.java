import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {

    static final double maxVel = 60;
    static final double maxAccel = 60;
    static final double maxAngVel = Math.toRadians(180);
    static final double maxAngAccel = Math.toRadians(180);
    static final double trackWidth = 12;

    static final double robotLength = 15;
    static final double robotWidth = 18;

    static final DriveTrainType driveTrain = DriveTrainType.TANK;

    public static final class Field {
        public enum StartPose {
            BLUE_CLOSE(61.0, 40.0, 180.0),
            BLUE_FAR(-61.0, 16.0, 0.0),
            RED_CLOSE(61.0, -40.0, 180.0),
            RED_FAR(-61.0, -16.0, 0.0);

            public final double startXIn, startYIn, startHeadingDeg;

            StartPose(double xIn, double yIn, double headingDeg) {
                this.startXIn = xIn;
                this.startYIn = yIn;
                this.startHeadingDeg = headingDeg;
            }

            public Pose2d pose() {
                return poseUser(startXIn, startYIn, startHeadingDeg);
            }
        }
    }

    static Pose2d poseUser(double xForward, double yLeft, double headingDegUser) {
        return new Pose2d(xForward, yLeft, Math.toRadians(headingDegUser));
    }

    static RoadRunnerBotEntity buildBot(MeepMeep meepMeep) {
        return new DefaultBotBuilder(meepMeep).setConstraints(maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth).setDriveTrainType(driveTrain).setDimensions(robotLength, robotWidth).build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity SoloRedClose = buildBot(meepMeep);
        RoadRunnerBotEntity SoloBlueClose = buildBot(meepMeep);
        RoadRunnerBotEntity SoloRedFar = buildBot(meepMeep);
        RoadRunnerBotEntity SoloBlueFar = buildBot(meepMeep);

        // XXX: Solo - Blue Close
        SoloBlueClose.runAction(SoloBlueClose.getDrive().actionBuilder(Field.StartPose.BLUE_CLOSE.pose())
                .setReversed(false)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .lineToY(12)
                .setReversed(false)
                .splineTo(new Vector2d(-12, 24), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-36, 24), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .splineTo(new Vector2d(12, 12), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-12, 56), Math.toRadians(60))
                .build());

        // XXX: Solo - Red Close
        SoloRedClose.runAction(SoloRedClose.getDrive().actionBuilder(Field.StartPose.RED_CLOSE.pose())
                .setReversed(false)
                .splineTo(new Vector2d(12, -24), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .lineToY(-12)
                .setReversed(false)
                .splineTo(new Vector2d(-12, -24), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .splineTo(new Vector2d(12, -12), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .splineTo(new Vector2d(12, -12), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(-12, -56), Math.toRadians(300))
                .build());

        // XXX: Solo - Blue Far
        SoloBlueFar.runAction(SoloBlueFar.getDrive().actionBuilder(Field.StartPose.BLUE_FAR.pose())
                .setReversed(false)
                .splineTo(new Vector2d(-36, 24), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(-12, 24), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(12, 32), Math.toRadians(90))
                .lineToY(54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                .build());

        // XXX: Solo - Red Far
        SoloRedFar.runAction(SoloRedFar.getDrive().actionBuilder(Field.StartPose.RED_FAR.pose())
                .setReversed(false)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(-12, -24), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                .setReversed(false)
                .splineTo(new Vector2d(12, -32), Math.toRadians(270))
                .lineToY(-54)
                .setReversed(true)
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
                .build());


        Image img = null;
        try {
            img = ImageIO.read(new File("MeepMeepTesting/src/main/java/DecodeFieldMap.png"));
        } catch (IOException ignored) {
        }
        assert img != null;
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(SoloBlueClose)
                .addEntity(SoloBlueFar)
                .addEntity(SoloRedClose)
                .addEntity(SoloRedFar)
                .start();
    }
}