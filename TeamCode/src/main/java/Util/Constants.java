package Util;

public final class Constants {

    public static final class Drive {
        public static final String frontLeft = "fL";
        public static final String frontRight = "fR";
        public static final String backLeft = "bL";
        public static final String backRight = "bR";

        public static final double MAX_WHEEL_RPM = 435;

        // TODO: TUNE THESE VALUES
        public static final double ROTATE_GAIN = 0.0350;
        public static final double MAX_ROTATE = 0.80;
        public static final double ARTIFACT_AIM_DEADBAND_DEG = 2.5;
    }

    public static final class PinPoint {
        public static final String PinPoint = "pinpoint";

        public static final boolean X_REVERSED = true;
        public static final boolean Y_REVERSED = false;

        public static final double X_OFFSET_MM = -176;
        public static final double Y_OFFSET_MM = -66;

        public static final double START_X_IN = 0;
        public static final double START_Y_IN = 0;
        public static final double START_HEADING_DEG = 0;
    }

    public static final class Elevator {
        public static final String gearShift = "gS";
        public static final String elevatorLeft = "eL";
        public static final String elevatorRight = "eR";

        public static final double GEAR_SHIFT_DEFAULT = 0.5;
        public static final double ELEVATOR_LEFT_DEFAULT = 0.5;
        public static final double ELEVATOR_RIGHT_DEFAULT = 0.5;

        public static final double GEAR_SHIFT_PRESET = 0.55;
        public static final double ELEVATOR_LEFT_PRESET = 1.0;
        public static final double ELEVATOR_RIGHT_PRESET = 0.0;
    }

    public static final class Intake {
        public static final String rollerIntake = "rI";

        public static final double INTAKE_POWER = 1.0;
        public static final double OUTTAKE_POWER = -1.0;
    }

    public static final class Turret {
        public static final String turretRotation = "tR";

        public static final double TurretTicksPerRev = 145.1;
        public static final double TurretDriver = 24.0;
        public static final double TurretDriven = 100.0;
        public static final double TurretOutputToTurretRatio = TurretDriver / TurretDriven;
        public static final double TurretTicksPerTurretRev = TurretTicksPerRev / TurretOutputToTurretRatio;
        public static final double TurretDegPerTick = 360.0 / TurretTicksPerTurretRev;

        public static final double TurretMinDeg = -180.0;
        public static final double TurretMaxDeg = 180.0;
        public static final double LimitGuard = 0.25;

        // TODO: TUNE THESE VALUES
        public static final double QuickKp = 0.0625;
        public static final double QuickMaxPower = 1.0;

        public static final double QuickMinPower = 0.05;
        public static final double QuickSlewPerSec = 7.5;
        public static final double QuickKfTurnRate = 0.0;

        public static final double PreciseKp = 0.020;
        public static final double PreciseMaxPower = 0.35;

        public static final double QuickDeadband = 5.0;
        public static final double PreciseDeadband = 1.0;

        public static final double SwitchDeadband = 15.0;
        public static final double LostTargetTimeout = 0.20;
    }

    public static final class Flywheel {
        public static final String Flywheel1 = "fW1";
        public static final String Flywheel2 = "fW2";

        public static final double TICKS_PER_REV = 28.0;

        // TODO: GET VALUES
        public static final double A = 0.0;
        public static final double B = 0.0;
        public static final double C = 0.0;
        public static final double M = 0.0;
        public static final double R = 0.0;

        public static final double MIN_RPS = 50.0;
        public static final double MAX_RPS = 100.0;

        public static final double F = 12.5;
        public static final double P = 30.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static final class Releases {
        public static final String artifactRelease = "aR";

        public static final double HOLD = 0.5;
        public static final double RELEASE = 0.75;
    }

    public static final class Vision {
        public static final String LIMELIGHT_NAME = "limelight";
        public static final int DEFAULT_PIPELINE = 0;
        public static final int ARTIFACT_PIPELINE = DEFAULT_PIPELINE;

        public static final String TURRET_CAM_NAME = "TurretCam";
        public static final int RESOLUTION_WIDTH = 1280;
        public static final int RESOLUTION_HEIGHT = 720;

        public static final int BLUE_TAG_ID = 20;
        public static final int RED_TAG_ID = 24;
    }

    public static final class Field {
        public static final double GOAL_X = 66.0;
        public static final double RED_GOAL_Y = -66.0;
        public static final double BLUE_GOAL_Y = 66.0;

        public static final double RED_CLOSE_X = 60.0;
        public static final double BLUE_CLOSE_X = 60.0;
        public static final double RED_CLOSE_Y = -56.0;
        public static final double BLUE_CLOSE_Y = 56.0;
        public static final double RED_CLOSE_HEADING_DEG = 180.0;
        public static final double BLUE_CLOSE_HEADING_DEG = 180.0;
        public static final double RED_FAR_X = -58.0;
        public static final double BLUE_FAR_X = -58.0;
        public static final double RED_FAR_Y = -20.0;
        public static final double BLUE_FAR_Y = 20.0;
        public static final double RED_FAR_HEADING_DEG = 0.0;
        public static final double BLUE_FAR_HEADING_DEG = 0.0;

        public enum StartPose {
            RED_CLOSE(RED_CLOSE_X, RED_CLOSE_Y, RED_CLOSE_HEADING_DEG, Constants.Vision.RED_TAG_ID),
            RED_FAR(RED_FAR_X, RED_FAR_Y, RED_FAR_HEADING_DEG, Constants.Vision.RED_TAG_ID),
            BLUE_CLOSE(BLUE_CLOSE_X, BLUE_CLOSE_Y, BLUE_CLOSE_HEADING_DEG, Constants.Vision.BLUE_TAG_ID),
            BLUE_FAR(BLUE_FAR_X, BLUE_FAR_Y, BLUE_FAR_HEADING_DEG, Constants.Vision.BLUE_TAG_ID);

            public final double startXIn, startYIn, startHeadingDeg;
            public final int trackedTagId;

            StartPose(double xIn, double yIn, double headingDeg, int tagId) {
                this.startXIn = xIn;
                this.startYIn = yIn;
                this.startHeadingDeg = headingDeg;
                this.trackedTagId = tagId;
            }

            public boolean isRed() {
                return trackedTagId == Constants.Vision.RED_TAG_ID;
            }

            public boolean isBlue() {
                return trackedTagId == Constants.Vision.BLUE_TAG_ID;
            }
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
            int buffer = 6;

            // Allowed Zones
            // Triangle 1: (-72, 24), (-48, 0), (-72, -24)
            boolean zone1 = inTriangle(
                    x, y,
                    -72 - buffer,  24 + buffer,
                    -48 + buffer,   0,
                    -72 - buffer, -24 - buffer
            );

            // Triangle 2: (72, 72), (0, 0), (72, -72)
            boolean zone2 = inTriangle(
                    x, y,
                     72 + buffer,  72 + buffer,
                      0 - buffer,   0,
                     72 + buffer, -72 - buffer
            );

            boolean inAllowed = zone1 || zone2;

            // Negated Zones
            // Triangle A: (24, 72), (72, 72), (72, 24)
            boolean zoneA = inTriangle(
                    x, y,
                     24 - buffer,  72 + buffer,
                     72 + buffer,  72 + buffer,
                     72 + buffer,  24 - buffer
            );

            // Triangle B: (24, -72), (72, -72), (72, -24)
            boolean zoneB = inTriangle(
                    x, y,
                     24 - buffer, -72 - buffer,
                     72 + buffer, -72 - buffer,
                     72 + buffer, -24 + buffer
            );

            boolean inNoShoot = zoneA || zoneB;

            return inAllowed && !inNoShoot;
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

        public enum Alliance { RED , BLUE }

        private static Alliance currentAlliance;

        public static void setAlliance(Alliance alliance) {
            currentAlliance = alliance;
        }

        public static Alliance getAlliance() {
            return currentAlliance;
        }
    }
}