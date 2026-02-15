package OpModes;

import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.exec;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.loop;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.sequence;
import static dev.frozenmilk.dairy.mercurial.continuations.Continuations.waitUntil;
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
public final class TeleOp {
    private static Mercurial.RegisterableProgram buildTeleOp(Constants.Field.Alliance alliance, boolean telemetry) {
        return Mercurial.teleop(linsane -> {

            // Hardware Init
            Drive.INSTANCE.setResetPinPointOnInit(false);
            Drive.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Vision.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            if (Vision.INSTANCE.getLimelight() != null) {
                Vision.INSTANCE.getLimelight().pipelineSwitch(Constants.Vision.ARTIFACT_PIPELINE);
            }
            Constants.Field.setAlliance(alliance);
            Vision.INSTANCE.setTrackedTag(alliance == Constants.Field.Alliance.RED ? Constants.Vision.RED_TAG_ID : Constants.Vision.BLUE_TAG_ID);
            Intake.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Flywheel.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Release.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Elevator.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());
            Turret.INSTANCE.init(linsane.hardwareMap(), linsane.telemetry());

            // Main Loop
            linsane.schedule(sequence(
                    waitUntil(linsane::inLoop),
                    loop(exec(() -> {
                        Vision.INSTANCE.update();
                        Drive.INSTANCE.updateOdometry();
                        Turret.INSTANCE.autoAimTurret(Drive.INSTANCE.getHeading(), Constants.Field.computeGoalHeadingDeg(Drive.INSTANCE.getX(), Drive.INSTANCE.getY(), alliance));

                        // Intake
                        if (linsane.gamepad1().right_bumper) {
                            Intake.INSTANCE.setMode(Intake.Mode.INTAKE);
                        } else if (linsane.gamepad1().left_bumper) {
                            Intake.INSTANCE.setMode(Intake.Mode.EJECT);
                        } else {
                            Intake.INSTANCE.setMode(Intake.Mode.IDLE);
                        }

                        // Drive
                        double driveCmd = -linsane.gamepad1().left_stick_y;
                        double turnCmd;
                        boolean intakeActive = (Intake.INSTANCE.getMode() == Intake.Mode.INTAKE);

                        if (intakeActive) {
                            if (Vision.INSTANCE.hasArtifact()) {
                                double tx = Vision.INSTANCE.getTX();
                                if (Math.abs(tx) <= Constants.Drive.ARTIFACT_AIM_DEADBAND_DEG) {
                                    turnCmd = 0.0;
                                } else {
                                    double assist = tx * Constants.Drive.ROTATE_GAIN;
                                    if (assist > Constants.Drive.MAX_ROTATE)
                                        assist = Constants.Drive.MAX_ROTATE;
                                    if (assist < -Constants.Drive.MAX_ROTATE)
                                        assist = -Constants.Drive.MAX_ROTATE;
                                    turnCmd = assist;
                                }
                            } else {
                                turnCmd = linsane.gamepad1().right_stick_x;
                            }
                        } else {
                            turnCmd = linsane.gamepad1().right_stick_x;
                        }

                        Drive.INSTANCE.drive(driveCmd, turnCmd);

                        // Flywheel
                        if (linsane.gamepad1().right_trigger > 0.05) {
                            Flywheel.INSTANCE.enableAutoRange();
                            Flywheel.INSTANCE.apply();
                        } else {
                            Flywheel.INSTANCE.stop();
                        }

                        Intake.INSTANCE.apply();
                        Flywheel.INSTANCE.apply();
                        Elevator.INSTANCE.updateRise();
                        Release.INSTANCE.update();

                        // Telemetry
                        if (telemetry) {
                            DefaultTelemetry.INSTANCE.update(linsane.telemetry());
                        }
                    }))
            ));

            // Release
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().a),
                    exec(() -> {
                        if (Constants.Field.inShootZone(Drive.INSTANCE.getX(), Drive.INSTANCE.getY())) {
                            Release.INSTANCE.toggle();
                        }
                    })
            );

            // Elevator
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().options && linsane.gamepad1().share),
                    exec(Elevator.INSTANCE::applyPreset)
            );
            linsane.bindSpawn(linsane.risingEdge(() -> linsane.gamepad1().dpad_up),
                    exec(Elevator.INSTANCE::startRise)
            );

            // Shut Off
            linsane.dropToScheduler();
        });
    }

    public static final Mercurial.RegisterableProgram DEBUG = buildTeleOp(Constants.Field.Alliance.RED, true);
    public static final Mercurial.RegisterableProgram RED = buildTeleOp(Constants.Field.Alliance.RED, false);
    public static final Mercurial.RegisterableProgram BLUE = buildTeleOp(Constants.Field.Alliance.BLUE, false);
}