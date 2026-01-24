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
import SubSystems.Vision;
import Util.Constants;
import dev.frozenmilk.dairy.mercurial.ftc.Mercurial;

@SuppressWarnings("unused")
public final class Auton {
    // XXX: Red Far
    public static final Mercurial.RegisterableProgram RedFar = Mercurial.autonomous(ctx -> {
        // Hardware Init
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Vision.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.RED_PIPELINE);
        }
        Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Release.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Elevator.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

        // Background Update Loop
        ctx.schedule(loop(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> {
                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();
                    Release.INSTANCE.update();
                    DefaultTelemetry.INSTANCE.update(ctx.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP
                })
        )));

        // Autonomous Sequence
        ctx.schedule(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> { /* INITIAL SETUP placeholder */ }),
                exec(() -> { /* MOVEMENT STEP 1 placeholder */ }),
                exec(() -> { /* MECHANISM STEP placeholder */ }),
                exec(() -> { /* VISION BRANCH placeholder */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        ctx.dropToScheduler();
    });

    // XXX: Red Close
    public static final Mercurial.RegisterableProgram RedClose = Mercurial.autonomous(ctx -> {
        // Hardware Init
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Vision.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.RED_PIPELINE);
        }
        Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Release.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Elevator.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

        // Background Update Loop
        ctx.schedule(loop(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> {
                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();
                    Release.INSTANCE.update();
                    DefaultTelemetry.INSTANCE.update(ctx.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP
                })
        )));

        // Autonomous Sequence
        ctx.schedule(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> { /* INITIAL SETUP placeholder */ }),
                exec(() -> { /* MOVEMENT STEP 1 placeholder */ }),
                exec(() -> { /* MECHANISM STEP placeholder */ }),
                exec(() -> { /* VISION BRANCH placeholder */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        ctx.dropToScheduler();
    });

    // XXX: Blue Far
    public static final Mercurial.RegisterableProgram BlueFar = Mercurial.autonomous(ctx -> {
        // Hardware Init
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Vision.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.BLUE_PIPELINE);
        }
        Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Release.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Elevator.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

        // Background Update Loop
        ctx.schedule(loop(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> {
                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();
                    Release.INSTANCE.update();
                    DefaultTelemetry.INSTANCE.update(ctx.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP
                })
        )));

        // Autonomous Sequence
        ctx.schedule(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> { /* INITIAL SETUP placeholder */ }),
                exec(() -> { /* MOVEMENT STEP 1 placeholder */ }),
                exec(() -> { /* MECHANISM STEP placeholder */ }),
                exec(() -> { /* VISION BRANCH placeholder */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        ctx.dropToScheduler();
    });

    // XXX: Blue Close
    public static final Mercurial.RegisterableProgram BlueClose = Mercurial.autonomous(ctx -> {
        // Hardware Init
        Drive.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Vision.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        if (Vision.INSTANCE.getLimelight() != null) {
            Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.BLUE_PIPELINE);
        }
        Intake.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Flywheel.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Release.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());
        Elevator.INSTANCE.init(ctx.hardwareMap(), ctx.telemetry());

        // Background Update Loop
        ctx.schedule(loop(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> {
                    Vision.INSTANCE.update();
                    Drive.INSTANCE.updateOdometry();
                    Intake.INSTANCE.apply();
                    Flywheel.INSTANCE.apply();
                    Elevator.INSTANCE.updateRise();
                    Release.INSTANCE.update();
                    DefaultTelemetry.INSTANCE.update(ctx.telemetry()); // TODO: DEBUGGING ONLY - DISABLE FOR COMP
                })
        )));

        // Autonomous Sequence
        ctx.schedule(sequence(
                waitUntil(ctx::inLoop),
                exec(() -> { /* INITIAL SETUP placeholder */ }),
                exec(() -> { /* MOVEMENT STEP 1 placeholder */ }),
                exec(() -> { /* MECHANISM STEP placeholder */ }),
                exec(() -> { /* VISION BRANCH placeholder */ }),

                // End Auton
                exec(() -> {
                    Drive.INSTANCE.stop();
                    Intake.INSTANCE.stop();
                    Flywheel.INSTANCE.stop();
                    Release.INSTANCE.cancel();
                })
        ));

        // Shut Off
        ctx.dropToScheduler();
    });
}