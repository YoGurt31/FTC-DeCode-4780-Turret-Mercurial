package Util;
public final class Constants {

    public static final class Drive {
        public static final String frontLeft = "fL";
        public static final String frontRight = "fR";
        public static final String backLeft = "bL";
        public static final String backRight = "bR";

        public static final double MAX_WHEEL_RPM = 435;

        public static final double ROTATE_GAIN = 0.0350;
        public static final double MAX_ROTATE = 0.80;
        public static final double ARTIFACT_AIM_DEADBAND_DEG = 3.0;
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
        public static final double LimitGuard = 0.5;

        // TODO: TUNE THESE VALUES
        public static final double QuickKp = 0.020;
        public static final double QuickMaxPower = 1.0;
        public static final double QuickDeadband = 5.0;

        public static final double PreciseKp = 0.020;
        public static final double PreciseMaxPower = 0.35;
        public static final double PreciseDeadband = 1.0;

        public static final double SwitchDeadband = 15.0;
        public static final double LostTargetTimeout = 0.20;
    }

    public static final class Flywheel {
        public static final String Flywheel1 = "fW1";
        public static final String Flywheel2 = "fW2";

        public static final double TICKS_PER_REV = 28.0;

        // TODO: CREATE EQUATION FOR VARIABLE RPS
        public static final double FAR_TARGET_RPS = 52.5;
        public static final double CLOSE_TARGET_RPS = 46.5;

        // TODO: TUNE THESE VALUES
        public static final double F = 18.0;
        public static final double P = 2.25 * F;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    public static final class Releases {
        public static final String artifactRelease = "aR";

        public static final double HOLD = 0.5;
        public static final double RELEASE = 0.0;

        public static final long GATE_OPEN_MS = 100;
        public static final long SHOT_TOTAL_MS = 1500;
    }

//    public static final class Vision {
//        public static final String LIMELIGHT_NAME = "limelight";
//
//        public static final int DEFAULT_PIPELINE = 0;
//        public static final int ARTIFACT_PIPELINE = DEFAULT_PIPELINE;
//
//        public static final String TURRET_CAM_NAME = "TurretCam";
//        public static final double TAG_AREA_THRESHOLD = 0.80;
//
//        public static final int BLUE_TAG_ID = 20;
//        public static final int RED_TAG_ID = 24;
//    }

//    public static final class Field {
//        public static final double GOAL_Y = 60.0;
//        public static final double RED_GOAL_X = 56.0;
//        public static final double BLUE_GOAL_X = -56.0;
//
//        public enum StartPose {
//            RED_CLOSE(56, 60, 180.0, Constants.Vision.RED_TAG_ID),
//            RED_FAR(12, -60, 0.0, Constants.Vision.RED_TAG_ID),
//            BLUE_CLOSE(-56, 60, 180.0, Constants.Vision.BLUE_TAG_ID),
//            BLUE_FAR(-12, -60, 0.0, Constants.Vision.BLUE_TAG_ID);
//
//            public final double startXIn, startYIn, startHeadingDeg;
//            public final int trackedTagId;
//
//            StartPose(double xIn, double yIn, double headingDeg, int tagId) {
//                this.startXIn = xIn;
//                this.startYIn = yIn;
//                this.startHeadingDeg = headingDeg;
//                this.trackedTagId = tagId;
//            }
//
//            public boolean isRed() {
//                return trackedTagId == Constants.Vision.RED_TAG_ID;
//            }
//
//            public boolean isBlue() {
//                return trackedTagId == Constants.Vision.BLUE_TAG_ID;
//            }
//        }
//
//        public static double computeGoalHeadingDeg(double robotX, double robotY, int trackedTagId) {
//            double goalX = (trackedTagId == Constants.Vision.RED_TAG_ID) ? Constants.Field.RED_GOAL_X : Constants.Field.BLUE_GOAL_X;
//            double goalY = Constants.Field.GOAL_Y;
//            return Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
//        }
//    }
}