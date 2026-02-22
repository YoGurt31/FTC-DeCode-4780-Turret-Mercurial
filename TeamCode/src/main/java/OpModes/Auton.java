package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import RoadRunner.TankDrive;
import dev.frozenmilk.dairy.mercurial.continuations.Closure;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import SubSystems.DefaultTelemetry;
import SubSystems.Drive;
import SubSystems.Elevator;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;

@SuppressWarnings("unused")
public final class Auton {
    private interface AutoBuilder {
        Closure build(AutoRuntime rt);
    }

    private static final class AutoRuntime {
        private final Constants.Field.StartPose startPose;
        private final TankDrive rr;
        private Action action;

        private AutoRuntime(Constants.Field.StartPose startPose, TankDrive rr) {
            this.startPose = startPose;
            this.rr = rr;
        }

        TankDrive rr() {
            return rr;
        }

        void start(Action a) {
            this.action = a;
        }

        boolean hasAction() {
            return action != null;
        }

        boolean stepAction() {
            if (action == null) return false;
            TelemetryPacket packet = new TelemetryPacket();
            boolean keepRunning = action.run(packet);
            if (!keepRunning) action = null;
            return true;
        }

        Closure waitForAction() {
            return waitUntil(() -> action == null);
        }

        Constants.Field.StartPose startPose() {
            return startPose;
        }
    }

    private static Mercurial.RegisterableProgram buildAuto(String name, Constants.Field.StartPose startPose, Constants.Field.Alliance alliance, int trackedTagId, AutoBuilder auton) {
        return Mercurial.autonomous(name, linsane -> {

            Drive.INSTANCE.setResetPinPointOnInit(false);
            Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Drive.INSTANCE.setPose(startPose.startXIn, startPose.startYIn, startPose.startHeadingDeg);

            Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            if (Vision.INSTANCE.getLimelight() != null) {
                Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
            }

            Constants.Field.setAlliance(alliance);
            Vision.INSTANCE.setTrackedTag(trackedTagId);

            Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.zeroTurret();

            TankDrive rr = new TankDrive(linsane.hardwareMap(), new Pose2d(startPose.startXIn, startPose.startYIn, Math.toRadians(startPose.startHeadingDeg)));
            AutoRuntime rt = new AutoRuntime(startPose, rr);

            final boolean[] wasInZone = {false};

            linsane.schedule(loop(sequence(
                    waitUntil(linsane::inLoop),
                    exec(() -> {
                        Vision.INSTANCE.update();
                        Drive.INSTANCE.updateOdometry();

                        double goalHeading = Constants.Field.computeGoalHeadingDeg(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), Constants.Field.getAlliance());
                        Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), goalHeading);

                        Flywheel.INSTANCE.enableAutoRange();
                        Flywheel.INSTANCE.apply();

                        boolean inZone = Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY());

                        Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                        Intake.INSTANCE.apply();

                        if (!inZone) {
                            Release.INSTANCE.close();
                        } else {
                            if (!wasInZone[0]) {
                                Release.INSTANCE.open();
                            }
                        }
                        wasInZone[0] = inZone;

                        Elevator.INSTANCE.updateRise();
                        Release.INSTANCE.update();

                        DefaultTelemetry.INSTANCE.update(linsane.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP

                        rt.stepAction();
                    })
            )));

            linsane.schedule(sequence(
                    waitUntil(linsane::inLoop),
                    auton.build(rt),
                    exec(() -> {
                        Drive.INSTANCE.stop();
                        Intake.INSTANCE.stop();
                        Flywheel.INSTANCE.stop();
                        Release.INSTANCE.close();
                        rr.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    })
            ));

            linsane.dropToScheduler();
        });
    }


    // XXX: Solo Autons
    public static final Mercurial.RegisterableProgram SoloRedFar = buildAuto(
            "Solo - Red Far",
            Constants.Field.StartPose.RED_FAR,
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram SoloBlueFar = buildAuto(
            "Solo - Blue Far",
            Constants.Field.StartPose.BLUE_FAR,
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram SoloRedClose = buildAuto(
            "Solo - Red Close",
            Constants.Field.StartPose.RED_CLOSE,
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, 12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .lineToX(12)
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram SoloBlueClose = buildAuto(
            "Solo - Blue Close",
            Constants.Field.StartPose.BLUE_CLOSE,
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, 12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .lineToX(-12)
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );


    // XXX: Ramp Autons
    public static final Mercurial.RegisterableProgram RampRedFar = buildAuto(
            "Ramp - Red Far",
            Constants.Field.StartPose.RED_FAR,
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -36), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, -60), Math.toRadians(180))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram RampBlueFar = buildAuto(
            "Ramp - Blue Far",
            Constants.Field.StartPose.BLUE_FAR,
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -36), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, -60), Math.toRadians(0))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram RampRedClose = buildAuto(
            "Ramp - Red Close",
            Constants.Field.StartPose.RED_CLOSE,
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(36, 12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .lineToX(12)
                            .setReversed(false)
                            .splineTo(new Vector2d(36, -12), Math.toRadians(0))
                            .lineToX(54)
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .setReversed(false)
                            .splineTo(new Vector2d(60, -8), Math.toRadians(30))
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 12), Math.toRadians(180))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );

    public static final Mercurial.RegisterableProgram RampBlueClose = buildAuto(
            "Ramp - Blue Close",
            Constants.Field.StartPose.BLUE_CLOSE,
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            rt -> sequence(
                    exec(() -> rt.start(rt.rr().actionBuilder(new Pose2d(rt.startPose().startXIn, rt.startPose().startYIn, Math.toRadians(rt.startPose().startHeadingDeg)))
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, 12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .lineToX(-12)
                            .setReversed(false)
                            .splineTo(new Vector2d(-36, -12), Math.toRadians(180))
                            .lineToX(-54)
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .setReversed(false)
                            .splineTo(new Vector2d(-60, -8), Math.toRadians(150))
                            .setReversed(true)
                            .splineTo(new Vector2d(-12, 12), Math.toRadians(0))
                            .build()
                    )),
                    rt.waitForAction()
            )
    );
}