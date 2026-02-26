package Util;

import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public final class Constants {

    @Configurable
    public static final class Drive {
        public static final String FRONT_LEFT = "fL";
        public static final String FRONT_RIGHT = "fR";
        public static final String BACK_LEFT = "bL";
        public static final String BACK_RIGHT = "bR";

        public static final DcMotorSimple.Direction LEFT_DIR = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction RIGHT_DIR = DcMotorSimple.Direction.FORWARD;

        // TODO: RETUNE
        public static double ROTATE_GAIN = 0.02;
        public static double MAX_ROTATE = 0.5;
        public static double ARTIFACT_AIM_DEADBAND_DEG = 5.0;
    }

    public static final class PinPoint {
        public static final String PINPOINT = "pinpoint";

        public static final boolean X_REVERSED = true;
        public static final boolean Y_REVERSED = false;

        // TODO: GET VALUES
        public static final double X_OFFSET_MM = 102.00;
        public static final double Y_OFFSET_MM = -34.348;

        public static final double START_X_IN = 0;
        public static final double START_Y_IN = 0;
        public static final double START_HEADING_DEG = 0;
    }

    @Configurable
    public static final class Relocalize {
        public static final double SHOOT_TRIGGER_DB = 0.10;
        public static final double STATIONARY_STICK_DB = 0.05;
        public static final long STATIONARY_TIME_MS = 750;
        public static final long COOLDOWN_MS = 250;

        public static final double METERS_TO_IN = 39.3701;

        // TODO: DETERMINE CONVERSIONS
        public static int LL_X_TO_PP_SIGN = +1;
        public static int LL_Y_TO_PP_SIGN = +1;
        public static boolean SWAP_XY = false;

        public static double LL_YAW_SIGN = 1.0;
        public static double LL_YAW_OFFSET_DEG = 0.0;

        public static final double MAX_DIST_JUMP_IN = 6;
        public static final double MAX_YAW_JUMP_DEG = 10;
    }

    @Configurable
    public static final class Elevator {
        public static final String GEAR_SHIFT = "gS";
        public static final String ELEVATOR_LEFT = "eL";
        public static final String ELEVATOR_RIGHT = "eR";

        public static double GEAR_SHIFT_DEFAULT = 0.5;
        public static double ELEVATOR_LEFT_DEFAULT = 0.5;
        public static double ELEVATOR_RIGHT_DEFAULT = 0.5;

        public static double GEAR_SHIFT_PRESET = 0.55;
        public static double ELEVATOR_LEFT_PRESET = 1.0;
        public static double ELEVATOR_RIGHT_PRESET = 0.0;
    }

    @Configurable
    public static final class Intake {
        public static final String ROLLER_INTAKE = "rI";

        public static final double INTAKE_POWER = 1.0;
        public static final double OUTTAKE_POWER = -1.0;

        public static double TRANSFER_SCALE_CLOSE = 1.0;
        public static double TRANSFER_SCALE_FAR = 0.8;
    }

    @Configurable
    public static final class Turret {
        public static final String TURRET_ROTATION = "tR";

        public static final double TURRET_TICKS_PER_REV = 384.5;
        public static final double TURRET_DRIVER = 24.0;
        public static final double TURRET_DRIVEN = 100.0;
        public static final double TURRET_OUTPUT_TO_TURRET_RATIO = TURRET_DRIVER / TURRET_DRIVEN;
        public static final double TURRET_TICKS_PER_TURRET_REV = TURRET_TICKS_PER_REV / TURRET_OUTPUT_TO_TURRET_RATIO;
        public static final double TURRET_DEG_PER_TICK = 360.0 / TURRET_TICKS_PER_TURRET_REV;

        public static final double TURRET_MIN_DEG = -180.0;
        public static final double TURRET_MAX_DEG = 180.0;
        public static final double LIMIT_GUARD = 0.1;

        // TODO: TUNE TURRET
        public static double TURRET_KP = 0.25;
        public static double TURRET_KD = 0.0;
        public static double TURRET_MIN_POWER = 0.0;
        public static double TURRET_MAX_POWER = 1.0;
        public static double TURRET_DEADBAND = 1;
    }

    @Configurable
    public static final class Flywheel {
        public static final String FLYWHEEL_1 = "fW1";
        public static final String FLYWHEEL_2 = "fW2";

        public static final double TICKS_PER_REV = 28.0;

        // TODO: GET VALUES
        public static final double A = 25.46382;
        public static final double B = 1198.15909;
        public static final double C = 1198.15909;
        public static final double M = 0.182786;
        public static final double R = 51.88337;

        // TODO: DETERMINE MIN RPS
        public static double MIN_RPS = 65.0;
        public static double MAX_RPS = 100.0;

        // TODO: RETUNE
        public static double F(double voltage) {
            if (voltage <= 1e-6) return 12.5;
            return 12.5 * (13.5 / voltage);
        }
        public static double P = 55.0;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    public static final class Releases {
        public static final String ARTIFACT_RELEASE = "aR";

        public static final double HOLD = 0.5;
        public static final double RELEASE = 0.75;
    }

    public static final class Vision {
        public static final String LIMELIGHT_NAME = "limelight";
        public static final int DEFAULT_PIPELINE = 0;
        public static final int LOCALIZATION_PIPELINE = DEFAULT_PIPELINE;
        public static final int ARTIFACT_PIPELINE = 1;
    }

    public static final class Field {
        public static final double GOAL_X = 66.0;
        public static final double RED_GOAL_Y = -66.0;
        public static final double BLUE_GOAL_Y = 66.0;

        public enum StartPose {
            BLUE_CLOSE(61.0, 40.0, 180.0),
            BLUE_FAR(-61.0, 16.0, 0.0),
            RED_CLOSE(61.0, -40.0, 180.0),
            RED_FAR(-61.0, -16.0, 0.0);

            public final double START_X_IN, START_Y_IN, START_HEADING_DEG;

            StartPose(double xIn, double yIn, double headingDeg) {
                this.START_X_IN = xIn;
                this.START_Y_IN = yIn;
                this.START_HEADING_DEG = headingDeg;
            }
        }

        public static Pose2d predictPose(double x, double y, double headingRad, double vx, double vy, double flyTimeSec) {
            return new Pose2d(x + vx * flyTimeSec, y + vy * flyTimeSec, headingRad);
        }

        private static double wrapDeg(double deg) {
            while (deg > 180.0) deg -= 360.0;
            while (deg <= -180.0) deg += 360.0;
            return deg;
        }

        public static double computeGoalHeadingDeg(double robotX, double robotY, Alliance alliance) {
            double goalX = GOAL_X;
            double goalY = (alliance == Alliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;

            double dx = goalX - robotX;
            double dy = goalY - robotY;

            return wrapDeg(Math.toDegrees(Math.atan2(dy, dx)));
        }

        public static boolean inShootZone(double x, double y) {
            int buffer = 12;

            // Allowed Zones
            // Triangle 1: (-72, 24), (-48, 0), (-72, -24)
            boolean farZone = inTriangle(
                    x, y,
                    -72 - buffer, 24 + buffer,
                    -48 + buffer, 0,
                    -72 - buffer, -24 - buffer
            );

            // Triangle 2: (72, 72), (0, 0), (72, -72)
            boolean closeZone = inTriangle(
                    x, y,
                    72 + buffer, 72 + buffer,
                    0 - buffer, 0,
                    72 + buffer, -72 - buffer
            );

            boolean inAllowed = farZone || closeZone;

            // Negated Zones
            // Triangle A: (24, 72), (72, 72), (72, 24)
            boolean zoneBlue = inTriangle(
                    x, y,
                    24 - buffer, 72 + buffer,
                    72 + buffer, 72 + buffer,
                    72 + buffer, 24 - buffer
            );

            // Triangle B: (24, -72), (72, -72), (72, -24)
            boolean zoneRed = inTriangle(
                    x, y,
                    24 - buffer, -72 - buffer,
                    72 + buffer, -72 - buffer,
                    72 + buffer, -24 + buffer
            );

            boolean inNoShoot = zoneBlue || zoneRed;

            return inAllowed && !inNoShoot;
        }

        public static boolean inFarZone(double x, double y) {
            int buffer = 12;

            boolean farZone = inTriangle(
                    x, y,
                    -72 - buffer, 24 + buffer,
                    -48 + buffer, 0,
                    -72 - buffer, -24 - buffer
            );

            boolean inAllowed = farZone;

            boolean zoneBlue = inTriangle(
                    x, y,
                    24 - buffer, 72 + buffer,
                    72 + buffer, 72 + buffer,
                    72 + buffer, 24 - buffer
            );

            boolean zoneRed = inTriangle(
                    x, y,
                    24 - buffer, -72 - buffer,
                    72 + buffer, -72 - buffer,
                    72 + buffer, -24 + buffer
            );

            boolean inNoShoot = zoneBlue || zoneRed;

            return inAllowed && !inNoShoot;
        }

        public static boolean inCloseZone(double x, double y) {
            int buffer = 12;

            return inTriangle(
                    x, y,
                    72 + buffer, 72 + buffer,
                    0 - buffer, 0,
                    72 + buffer, -72 - buffer
            );
        }

        private static boolean inTriangle(double px, double py, double x1, double y1, double x2, double y2, double x3, double y3) {
            double d1 = sign(px, py, x1, y1, x2, y2);
            double d2 = sign(px, py, x2, y2, x3, y3);
            double d3 = sign(px, py, x3, y3, x1, y1);

            boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

            return !(hasNeg && hasPos);
        }

        private static double sign(double px, double py, double x1, double y1, double x2, double y2) {
            return (px - x2) * (y1 - y2) - (x1 - x2) * (py - y2);
        }

        public enum Alliance {RED, BLUE}

        private static Alliance CURRENT_ALLIANCE;

        public static void setAlliance(Alliance alliance) {
            CURRENT_ALLIANCE = alliance;
        }

        public static Alliance getAlliance() {
            return CURRENT_ALLIANCE;
        }

        public static double distanceToGoal() {
            return Math.hypot(Constants.Field.GOAL_X - SubSystems.Drive.INSTANCE.getX(), ((Constants.Field.getAlliance() == Constants.Field.Alliance.RED) ? Constants.Field.RED_GOAL_Y : Constants.Field.BLUE_GOAL_Y) - SubSystems.Drive.INSTANCE.getY());
        }

    }

    @Configurable
    public static final class Ballistic {
        // TODO: UPDATE LAUNCH ANGLE + TUNE SLIP
        public static final double LAUNCH_ANGLE_DEG = 35.0;
        public static final double FLYWHEEL_RADIUS_IN = (36.0 / 25.4);
        public static double SLIP = 0.85;
        private static final double EPS = 1e-6;

        public static double flyTime(double dist, double rps) {
            double speed = ((2.0 * Math.PI * FLYWHEEL_RADIUS_IN * rps) * (SLIP)) * Math.cos(Math.toRadians(LAUNCH_ANGLE_DEG));
            if (speed <= EPS) return 0.0;
            return Math.max(0.0, dist / speed);
        }
    }
}