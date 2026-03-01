package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import SubSystems.Drive;
import SubSystems.Elevator;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

import com.qualcomm.robotcore.util.Range;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import Util.Actions;
import Util.P2PTankController;

@SuppressWarnings("unused")
public final class Auton {
    private static Action runningAction = null;

    public static final Mercurial.RegisterableProgram RedFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.RED);
        Drive.INSTANCE.setPose(Constants.Field.StartPose.RED_FAR.START_X_IN, Constants.Field.StartPose.RED_FAR.START_Y_IN, Constants.Field.StartPose.RED_FAR.START_HEADING_DEG);

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        runningAction = null;

        // Background Update Loop
        linsane.schedule(loop(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> {
                    Drive.INSTANCE.updateOdometry();
                    if (Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY())) Release.INSTANCE.open();
                    else Release.INSTANCE.close();
                    Vision.INSTANCE.update();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Release.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();

                    if (runningAction != null) {
                        TelemetryPacket packet = new TelemetryPacket();
                        boolean keep = runningAction.run(packet);
                        if (!keep) runningAction = null;
                    }
                })
        )));

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> runningAction = buildMain(new P2PTankController(), Constants.Field.getAlliance())),
                waitUntil(() -> runningAction == null),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    public static final Mercurial.RegisterableProgram BlueFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(true);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Constants.Field.setAlliance(Constants.Field.Alliance.BLUE);
        Drive.INSTANCE.setPose(Constants.Field.StartPose.BLUE_FAR.START_X_IN, Constants.Field.StartPose.BLUE_FAR.START_Y_IN, Constants.Field.StartPose.BLUE_FAR.START_HEADING_DEG);

        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        runningAction = null;

        // Background Update Loop
        linsane.schedule(loop(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> {
                    Drive.INSTANCE.updateOdometry();
                    if (Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY())) Release.INSTANCE.open();
                    else Release.INSTANCE.close();
                    Vision.INSTANCE.update();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Release.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();

                    if (runningAction != null) {
                        TelemetryPacket packet = new TelemetryPacket();
                        boolean keep = runningAction.run(packet);
                        if (!keep) runningAction = null;
                    }
                })
        )));

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> runningAction = buildMain(new P2PTankController(), Constants.Field.getAlliance())),
                waitUntil(() -> runningAction == null),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.close();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    private static Action p2pTo(P2PTankController p2p, double xIn, double yIn, double headingDeg) {
        return new Action() {
            private boolean started;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    started = true;
                    p2p.beginAbs(xIn, yIn, headingDeg);
                }
                return p2p.step();
            }
        };
    }

    private static Action shootArtifactsAction(Constants.Field.Alliance alliance) {

        Runnable lockTurret = () -> {
            Turret.INSTANCE.lockTurretAt(0.0);
        };

        Action spinUpAndAlignUntilReady = Actions.until(
                () -> {
                    return Flywheel.INSTANCE.isReady() && (!(Vision.INSTANCE.hasTag()) || Math.abs(Vision.INSTANCE.getTX()) <= 1.0);
                },
                2.25,
                () -> {
                    lockTurret.run();

                    double turn = Range.clip((Vision.INSTANCE.getTX() * Constants.Drive.KP) * Constants.Drive.TX_SIGN, -Constants.Drive.MAX_TURN, Constants.Drive.MAX_TURN);

                    if (!Vision.INSTANCE.hasTag() || Math.abs(Vision.INSTANCE.getTX()) <= 1.0) {
                        turn = 0.0;
                    }

                    Drive.INSTANCE.drive(0.0, turn);

                    Flywheel.INSTANCE.enableAutoRange();
                    Flywheel.INSTANCE.apply();
                }
        );

        Runnable shootLoop = () -> {
            lockTurret.run();

            Flywheel.INSTANCE.enableAutoRange();
            Flywheel.INSTANCE.apply();

            if (Flywheel.INSTANCE.isReady() && ((!Vision.INSTANCE.hasTag()) || Math.abs(Vision.INSTANCE.getTX()) <= 1.0)) {
                Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            } else {
                Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                double turn = 0.0;
                if (Vision.INSTANCE.hasTag() && Math.abs(Vision.INSTANCE.getTX()) > 1.0) {
                    turn = Range.clip((Vision.INSTANCE.getTX() * Constants.Drive.KP) * Constants.Drive.TX_SIGN, -Constants.Drive.MAX_TURN, Constants.Drive.MAX_TURN);
                }
                Drive.INSTANCE.drive(0.0, turn);
            }

            Intake.INSTANCE.apply();

            if (Flywheel.INSTANCE.isReady() && ((!Vision.INSTANCE.hasTag()) || Math.abs(Vision.INSTANCE.getTX()) <= 1.0)) Drive.INSTANCE.drive(0.0, 0.0);
        };

        Action shot1 = Actions.runFor(0.15, shootLoop);
        Action gap1 = Actions.waitSeconds(1.50);

        Action shot2 = Actions.runFor(0.15, shootLoop);
        Action gap2 = Actions.waitSeconds(1.50);

        Action shot3 = Actions.runFor(0.15, shootLoop);

        Action cleanup = Actions.instant(() -> {
            Drive.INSTANCE.drive(0.0, 0.0);
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
            Flywheel.INSTANCE.stop();
            Turret.INSTANCE.stop();
        });

        return Actions.sequence(
                spinUpAndAlignUntilReady,
                shot1, gap1,
                shot2, gap2,
                shot3,
                cleanup
        );
    }

    private static Action intakeArtifactsAction() {
        Action intake = Actions.runFor(3.00, () -> {
            Release.INSTANCE.close();
            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
            Intake.INSTANCE.apply();
        });

        Action cleanup = Actions.instant(() -> {
            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
            Intake.INSTANCE.apply();
        });

        return Actions.sequence(intake, cleanup);
    }

    private static Action buildMain(P2PTankController p2p, Constants.Field.Alliance alliance) {
        Constants.Field.StartPose sp = (alliance == Constants.Field.Alliance.BLUE) ? Constants.Field.StartPose.BLUE_FAR : Constants.Field.StartPose.RED_FAR;

        Drive.INSTANCE.setPose(sp.START_X_IN, sp.START_Y_IN, sp.START_HEADING_DEG);

        return (alliance == Constants.Field.Alliance.BLUE) ? blueFar(p2p, alliance) : redFar(p2p, alliance);
    }

    // TODO: BlueFar Path
    private static Action blueFar(P2PTankController p2p, Constants.Field.Alliance alliance) {
        Action driveAndIntakeLine1 = Actions.parallel(
                Actions.sequence(
                        p2pTo(p2p, -36, 24, 90),
                        p2pTo(p2p, -36, 54, 90)
                ),
                intakeArtifactsAction()
        );
        Action driveAndIntakeHPZ = Actions.parallel(
                p2pTo(p2p, -60, 54, 90),
                intakeArtifactsAction()
        );

        return Actions.sequence(
                shootArtifactsAction(alliance),
                driveAndIntakeLine1,
                p2pTo(p2p, -60, 12, 180),
                shootArtifactsAction(alliance),
                driveAndIntakeHPZ,
                p2pTo(p2p, -60, 12, 180),
                shootArtifactsAction(alliance)
        );
    }

    // TODO: RedFar Path
    private static Action redFar(P2PTankController p2p, Constants.Field.Alliance alliance) {
        Action driveAndIntakeLine1 = Actions.parallel(
                Actions.sequence(
                        p2pTo(p2p, -36, -24, -90),
                        p2pTo(p2p, -36, -54, -90)
                ),
                intakeArtifactsAction()
        );
        Action driveAndIntakeHPZ = Actions.parallel(
                p2pTo(p2p, -60, -54, -90),
                intakeArtifactsAction()
        );

        return Actions.sequence(
                shootArtifactsAction(alliance),
                driveAndIntakeLine1,
                p2pTo(p2p, -60, -12, 180),
                shootArtifactsAction(alliance),
                driveAndIntakeHPZ,
                p2pTo(p2p, -60, -12, 180),
                shootArtifactsAction(alliance)
        );
    }
}