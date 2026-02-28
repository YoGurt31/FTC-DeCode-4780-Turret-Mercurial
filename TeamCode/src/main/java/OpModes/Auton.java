package OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoadRunner.TankDrive;
import SubSystems.DefaultTelemetry;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;

public final class Auton {
    private Auton() {}

    static final class InstantAction implements Action {
        private final Runnable r;
        private boolean ran;

        InstantAction(Runnable r) {
            this.r = r;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!ran) {
                ran = true;
                r.run();
            }
            return false;
        }
    }

    static abstract class BaseAuto extends LinearOpMode {
        protected abstract Constants.Field.StartPose startPoseDefined();
        protected abstract Constants.Field.Alliance alliance();
        protected abstract Action buildMain(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled);

        @Override
        public final void runOpMode() {
            Vision.INSTANCE.init(hardwareMap, telemetry);
            if (Vision.INSTANCE.getLimelight() != null) {
                Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
            }

            Constants.Field.setAlliance(alliance());

            Intake.INSTANCE.init(hardwareMap, telemetry);
            Flywheel.INSTANCE.init(hardwareMap, telemetry);
            Release.INSTANCE.init(hardwareMap, telemetry);
            Turret.INSTANCE.init(hardwareMap, telemetry);
            Turret.INSTANCE.zeroTurret();

            Constants.Field.StartPose sp = startPoseDefined();
            Pose2d startPose = new Pose2d(
                    sp.START_X_IN,
                    sp.START_Y_IN,
                    Math.toRadians(sp.START_HEADING_DEG)
            );

            TankDrive drive = new TankDrive(hardwareMap, startPose);

            final boolean[] intakeEnabled = { false };
            Action main = buildMain(drive, startPose, intakeEnabled);

            waitForStart();
            if (isStopRequested()) return;

            final boolean[] wasInZone = { false };

            Action running = main;
            while (opModeIsActive() && running != null) {
                Vision.INSTANCE.update();

                Pose2d pose = drive.localizer.getPose();
                double x = pose.position.x;
                double y = pose.position.y;
                double headingDeg = Math.toDegrees(pose.heading.toDouble());

                double goalHeadingDeg = Constants.Field.computeGoalHeadingDeg(x, y, Constants.Field.getAlliance());
                Turret.INSTANCE.autoAimTurret(headingDeg, goalHeadingDeg);

                Flywheel.INSTANCE.enableAutoRange();
                Flywheel.INSTANCE.apply();

                Intake.INSTANCE.setMode(intakeEnabled[0] ? Intake.Mode.INTAKE : Intake.Mode.IDLE);
                Intake.INSTANCE.apply();

                boolean inZone = Constants.Field.inShootZone(x, y);
                if (!inZone) {
                    Release.INSTANCE.close();
                } else {
                    if (!wasInZone[0]) {
                        Release.INSTANCE.open();
                    }
                }
                wasInZone[0] = inZone;
                Release.INSTANCE.apply();

                TelemetryPacket packet = new TelemetryPacket();
                boolean keepRunning = running.run(packet);
                if (!keepRunning) running = null;

                DefaultTelemetry.INSTANCE.update(telemetry);
                telemetry.update();
            }

            Intake.INSTANCE.stop();
            Flywheel.INSTANCE.stop();
            Release.INSTANCE.close();
        }
    }

    static final class Paths {
        private Paths() {}

        static Action redFar(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled) {
            return drive.actionBuilder(startPose)
                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, -60), Math.toRadians(180))

                    .setReversed(false)
                    .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(false)
                    .splineTo(new Vector2d(60, -8), Math.toRadians(30))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(false)
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                    .build();
        }

        static Action blueFar(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled) {
            return drive.actionBuilder(startPose)
                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, -60), Math.toRadians(0))

                    .setReversed(false)
                    .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(false)
                    .splineTo(new Vector2d(-60, -8), Math.toRadians(150))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(false)
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                    .build();
        }

        static Action redClose(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled) {
            return drive.actionBuilder(startPose)
                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(36, 12), Math.toRadians(0))
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .lineToX(12)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                    .lineToX(54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(60, -8), Math.toRadians(30))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                    .build();
        }

        static Action blueClose(TankDrive drive, Pose2d startPose, boolean[] intakeEnabled) {
            return drive.actionBuilder(startPose)
                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(-36, 12), Math.toRadians(180))
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .lineToX(-12)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                    .lineToX(-54)

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = true))

                    .setReversed(false)
                    .splineTo(new Vector2d(-60, -8), Math.toRadians(150))

                    .stopAndAdd(new InstantAction(() -> intakeEnabled[0] = false))

                    .setReversed(true)
                    .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                    .build();
        }
    }
}