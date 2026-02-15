package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

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
    private static Mercurial.RegisterableProgram buildAuto(String name, Constants.Field.Alliance alliance, int trackedTagId, Closure auton) {
        return Mercurial.autonomous(name, linsane -> {

            Drive.INSTANCE.setResetPinPointOnInit(true);
            Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

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

            linsane.schedule(loop(sequence(
                    waitUntil(linsane::inLoop),
                    exec(() -> {
                        Vision.INSTANCE.update();
                        Drive.INSTANCE.updateOdometry();
                        Intake.INSTANCE.apply();
                        Flywheel.INSTANCE.apply();
                        Elevator.INSTANCE.updateRise();
                        Release.INSTANCE.update();
                        DefaultTelemetry.INSTANCE.update(linsane.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP
                    })
            )));

            linsane.schedule(sequence(
                    waitUntil(linsane::inLoop),
                    auton,
                    exec(() -> {
                        Drive.INSTANCE.stop();
                        Intake.INSTANCE.stop();
                        Flywheel.INSTANCE.stop();
                        Release.INSTANCE.close();
                    })
            ));

            linsane.dropToScheduler();
        });
    }


    // XXX: Solo Autons
    public static final Mercurial.RegisterableProgram SoloRedFar = buildAuto(
            "Solo - Red Far",
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram SoloBlueFar = buildAuto(
            "Solo - Blue Far",
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram SoloRedClose = buildAuto(
            "Solo - Red Close",
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram SoloBlueClose = buildAuto(
            "Solo - Blue Close",
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );


    // XXX: CoOp Autons
    public static final Mercurial.RegisterableProgram CoOpRedFar = buildAuto(
            "CoOp - Red Far",
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram CoOpBlueFar = buildAuto(
            "CoOp - Blue Far",
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram CoOpRedClose = buildAuto(
            "CoOp - Red Close",
            Constants.Field.Alliance.RED,
            Constants.Vision.RED_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );

    public static final Mercurial.RegisterableProgram CoOpBlueClose = buildAuto(
            "CoOp - Blue Close",
            Constants.Field.Alliance.BLUE,
            Constants.Vision.BLUE_TAG_ID,
            sequence(
                    exec(() -> {
                        /* CODE LINES HERE */
                    })
            )
    );
}