package OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import RoadRunner.TankDrive;
import SubSystems.DefaultTelemetry;
import SubSystems.Drive;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;

public final class Auton {
    private Auton() {
    }

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

    static final class WaitAction implements Action {
        private final double seconds;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started;

        WaitAction(double seconds) {
            this.seconds = Math.max(0.0, seconds);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                started = true;
                timer.reset();
            }
            return timer.seconds() < seconds;
        }
    }

    static final class RunForAction implements Action {
        private final double seconds;
        private final Runnable onLoop;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started;

        RunForAction(double seconds, Runnable onLoop) {
            this.seconds = Math.max(0.0, seconds);
            this.onLoop = onLoop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                started = true;
                timer.reset();
            }
            if (onLoop != null) onLoop.run();
            return timer.seconds() < seconds;
        }
    }

    static final class UntilAction implements Action {
        interface Condition {
            boolean get();
        }

        private final Condition cond;
        private final double timeoutSec;
        private final Runnable onLoop;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean started;

        UntilAction(Condition cond, double timeoutSec, Runnable onLoop) {
            this.cond = cond;
            this.timeoutSec = Math.max(0.0, timeoutSec);
            this.onLoop = onLoop;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                started = true;
                timer.reset();
            }
            if (onLoop != null) onLoop.run();
            boolean done = (cond != null && cond.get());
            boolean timedOut = timer.seconds() >= timeoutSec;
            return !(done || timedOut);
        }
    }

    static final class SequenceAction implements Action {
        private final Action[] actions;
        private int i;

        SequenceAction(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actions == null || actions.length == 0) return false;
            while (i < actions.length) {
                Action a = actions[i];
                if (a == null) {
                    i++;
                    continue;
                }
                boolean keep = a.run(packet);
                if (keep) return true;
                i++;
            }
            return false;
        }
    }

    static final class ParallelAction implements Action {
        private final Action[] actions;

        ParallelAction(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean anyRunning = false;
            for (Action a : actions) {
                if (a != null && a.run(packet)) {
                    anyRunning = true;
                }
            }
            return anyRunning;
        }
    }

    static Action shootArtifactsAction(TankDrive drive, Constants.Field.Alliance alliance) {
        Runnable autoAim = () -> {
            if (drive == null) return;
            Pose2d pose = Constants.Field.predictPose(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Math.toRadians(Drive.INSTANCE.getHeading()), Drive.INSTANCE.getVx(), Drive.INSTANCE.getVy(), Constants.Ballistic.flyTime(Constants.Field.distanceToGoal(), Flywheel.INSTANCE.getTargetRps()));
            Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(pose.position.x, pose.position.y, alliance));
        };

        Action spinUpAndAimUntilReady = new UntilAction(
                () -> Flywheel.INSTANCE.isReady(),
                1.50,
                () -> {
                    autoAim.run();
                    Flywheel.INSTANCE.enableAutoRange();
                    Flywheel.INSTANCE.apply();
                }
        );

        Runnable shootLoop = () -> {
            autoAim.run();
            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();
            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            Intake.INSTANCE.apply();
        };

        Action shot1 = new RunForAction(0.15, shootLoop);
        Action gap1  = new WaitAction(0.5);

        Action shot2 = new RunForAction(0.15, shootLoop);
        Action gap2  = new WaitAction(0.5);

        Action shot3 = new RunForAction(0.15, shootLoop);

        Action cleanup = new InstantAction(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.stop();
            Turret.INSTANCE.stop();
        });

        return new SequenceAction(
                spinUpAndAimUntilReady,
                shot1, gap1,
                shot2, gap2,
                shot3,
                cleanup
        );
    }

    static Action intakeArtifactsAction() {
        Action intake = new RunForAction(3.00, () -> {
            Release.INSTANCE.close();
            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            Intake.INSTANCE.apply();
        });

        Action cleanup = new InstantAction(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        });

        return new SequenceAction(
                intake,
                cleanup
        );
    }

    static abstract class BaseAuto extends LinearOpMode {
        protected abstract Constants.Field.StartPose startPoseDefined();
        protected abstract Constants.Field.Alliance alliance();

        protected abstract Action buildMain(TankDrive drive, Pose2d startPose);

        @Override
        public final void runOpMode() {
            Vision.INSTANCE.init(hardwareMap, telemetry);
            if (Vision.INSTANCE.getLimelight() != null) {
                Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
            }

            Constants.Field.setAlliance(alliance());

            Drive.INSTANCE.setResetPinPointOnInit(true);
            Drive.INSTANCE.init(hardwareMap, telemetry);
            Intake.INSTANCE.init(hardwareMap, telemetry);
            Flywheel.INSTANCE.init(hardwareMap, telemetry);
            Release.INSTANCE.init(hardwareMap, telemetry);
            Turret.INSTANCE.init(hardwareMap, telemetry);
            Turret.INSTANCE.zeroTurret();

            Constants.Field.StartPose sp = startPoseDefined();
            Pose2d startPose = new Pose2d(sp.START_X_IN, sp.START_Y_IN, Math.toRadians(sp.START_HEADING_DEG));
            Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, Math.toRadians(sp.START_HEADING_DEG));
            Drive.INSTANCE.updateOdometry();

            TankDrive drive = new TankDrive(hardwareMap, startPose);

            Action main = buildMain(drive, startPose);

            waitForStart();
            if (isStopRequested()) return;

            Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, Math.toRadians(sp.START_HEADING_DEG));
            Drive.INSTANCE.updateOdometry();

            Action running = main;
            while (opModeIsActive() && running != null) {
                Drive.INSTANCE.updateOdometry();

                Vision.INSTANCE.update();
                Vision.INSTANCE.updateRobotYawDeg(Drive.INSTANCE.getHeading());

                Pose2d pose = drive.localizer.getPose();
                double x = pose.position.x;
                double y = pose.position.y;
                double headingDeg = Math.toDegrees(pose.heading.toDouble());

                if (Constants.Field.inShootZone(x, y)) {
                    Release.INSTANCE.open();
                } else {
                    Release.INSTANCE.close();
                }
                Release.INSTANCE.apply();

                TelemetryPacket packet = new TelemetryPacket();
                boolean keepRunning = running.run(packet);
                if (!keepRunning) running = null;

                DefaultTelemetry.INSTANCE.update(telemetry);
                telemetry.update();
            }

            Drive.INSTANCE.drive(0.0, 0.0);
            Intake.INSTANCE.stop();
            Flywheel.INSTANCE.stop();
            Turret.INSTANCE.stop();
            Release.INSTANCE.close();
        }
    }

    static final class Paths {
        private Paths() {
        }

//        static Action redFar(TankDrive drive, Pose2d startPose) {
//            return drive.actionBuilder(startPose)
//                    .setReversed(false)
//                    .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
//                    .lineToY(-54)
//                    .setReversed(true)
//                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//                    .setReversed(false)
//                    .splineTo(new Vector2d(-12, -24), Math.toRadians(270))
//                    .lineToY(-54)
//                    .setReversed(true)
//                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//                    .setReversed(false)
//                    .splineTo(new Vector2d(12, -32), Math.toRadians(270))
//                    .lineToY(-54)
//                    .setReversed(true)
//                    .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//                    .build();
//        }

        static Action blueFar(TankDrive drive, Pose2d startPose) {
            return drive.actionBuilder(startPose)
                    .stopAndAdd(Auton.shootArtifactsAction(drive, Constants.Field.Alliance.BLUE))
                    .setReversed(false)
                    .splineTo(new Vector2d(-36, 24), Math.toRadians(90))
                    .stopAndAdd(new ParallelAction(
                            drive.actionBuilder(new Pose2d(-36, 24, Math.toRadians(90)))
                                    .lineToY(54)
                                    .build(),
                            Auton.intakeArtifactsAction()
                    ))
                    .setReversed(true)
                    .splineTo(new Vector2d(-60, 12), Math.toRadians(180))
                    .stopAndAdd(Auton.shootArtifactsAction(drive, Constants.Field.Alliance.BLUE))
                    .setReversed(false)
                    .build();
        }
    }
}