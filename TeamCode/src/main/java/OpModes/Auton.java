package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;

import SubSystems.DefaultTelemetry;
import SubSystems.Drive;
import SubSystems.Elevator;
import SubSystems.Flywheel;
import SubSystems.Intake;
import SubSystems.Release;
import SubSystems.Turret;
import SubSystems.Vision;
import Util.Constants;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class Auton {
    // XXX: Red Far
    public static final Mercurial.RegisterableProgram RedFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(false);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
        }
        Vision.INSTANCE.setTrackedTag(Constants.Vision.RED_TAG_ID);
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
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

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> { /* CODE LINES HERE */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    // XXX: Red Close
    public static final Mercurial.RegisterableProgram RedClose = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(false);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
        }
        Vision.INSTANCE.setTrackedTag(Constants.Vision.RED_TAG_ID);
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
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

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> { /* CODE LINES HERE */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    // XXX: Blue Far
    public static final Mercurial.RegisterableProgram BlueFar = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(false);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
        }
        Vision.INSTANCE.setTrackedTag(Constants.Vision.BLUE_TAG_ID);
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
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

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> { /* CODE LINES HERE */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });

    // XXX: Blue Close
    public static final Mercurial.RegisterableProgram BlueClose = Mercurial.autonomous(linsane -> {

        // Hardware Init
        Drive.INSTANCE.setResetPinPointOnInit(false);
        Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
        }
        Vision.INSTANCE.setTrackedTag(Constants.Vision.BLUE_TAG_ID);
        Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
        Turret.INSTANCE.zeroTurret();

        // Background Update Loop
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

        // Autonomous Sequence
        linsane.schedule(sequence(
                waitUntil(linsane::inLoop),
                exec(() -> { /* CODE LINES HERE */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        linsane.dropToScheduler();
    });
}