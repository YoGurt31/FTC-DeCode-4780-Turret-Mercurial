package OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;

import SubSystems.DriveSystem;
import SubSystems.IntakeSystem;
import SubSystems.OuttakeSystem;
import SubSystems.VisionSystem;

import Commands.DriveCommands;
import Commands.IntakeCommands;
import Commands.OuttakeCommands;
import Commands.VisionCommands;

@TeleOp(name = "RED", group = "TeleOp")
@Mercurial.Attach
@DriveSystem.Attach
@IntakeSystem.Attach
@OuttakeSystem.Attach
@VisionSystem.Attach
public class TeleOpRed extends OpMode {

    private static final int RED_TARGET_ID = 24;

    private static final double AIM_ROTATE_GAIN = 0.025;
    private static final double AIM_MAX_ROTATE  = 0.75;

    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FAR_TARGET_RPS  = 52.5;
    private static final double READY_THRESHOLD_RPS = 2.5;

    private static final double LEFT_GATE_CLOSED  = 0.0;
    private static final double LEFT_GATE_OPEN    = 0.5;
    private static final double RIGHT_GATE_CLOSED = 0.5;
    private static final double RIGHT_GATE_OPEN   = 1.0;

    private static final double INTAKE_POWER = 1.0;
    private static final double SORT_LEFT_POWER  = 1.0;
    private static final double SORT_RIGHT_POWER = -1.0;
    private static final double POST_SHOT_FEED_S  = 3.0;

    private final DriveSystem drive = DriveSystem.INSTANCE;
    private final IntakeSystem intake = IntakeSystem.INSTANCE;
    private final OuttakeSystem outtake = OuttakeSystem.INSTANCE;
    private final VisionSystem vision = VisionSystem.INSTANCE;

    private Lambda driveCommand;
    private Lambda intakeTeleopCommand;

    private boolean aimbotActive() {
        return gamepad1.left_trigger >= 0.5;
    }

    private double aimbotRotate() {
        Double tx = VisionSystem.getTxIfSeeing(RED_TARGET_ID);
        if (tx == null) return gamepad1.right_stick_x;
        return Range.clip(tx * AIM_ROTATE_GAIN, -AIM_MAX_ROTATE, AIM_MAX_ROTATE);
    }

    private double avgFlywheelRps() {
        return OuttakeSystem.flywheelRpsAvg();
    }

    private void spinFlywheelToFar() {
        OuttakeSystem.setFlywheelRps(FAR_TARGET_RPS);
    }

    private void stopFlywheel() {
        OuttakeSystem.stopFlywheel();
    }

    @Override
    public void init() {
        BoundGamepad g1 = new BoundGamepad(new SDKGamepad(gamepad1));
        Mercurial.gamepad1(g1);

        BoundGamepad g2 = new BoundGamepad(new SDKGamepad(gamepad2));
        Mercurial.gamepad2(g2);

        VisionSystem.setPollRateHz(30);
        VisionSystem.setPipeline(vision.RED);

        OuttakeSystem.leftGate().setPosition(LEFT_GATE_CLOSED);
        OuttakeSystem.rightGate().setPosition(RIGHT_GATE_CLOSED);
        stopFlywheel();

        driveCommand = DriveCommands.teleOpDrive(
                () -> -gamepad1.left_stick_y,
                () -> {
                    if (aimbotActive() && VisionSystem.seesFiducialId(RED_TARGET_ID)) {
                        return aimbotRotate();
                    }
                    return gamepad1.right_stick_x;
                }
        );

        intakeTeleopCommand = new Lambda("IntakeTeleOp")
                .addRequirements(intake)
                .setExecute(() -> {
                    boolean lb = Mercurial.gamepad1().leftBumper().state();
                    boolean rb = Mercurial.gamepad1().rightBumper().state();

                    if (lb && !rb) {
                        IntakeSystem.intakeLeft(INTAKE_POWER, SORT_LEFT_POWER);
                    } else if (rb && !lb) {
                        IntakeSystem.intakeRight(INTAKE_POWER, SORT_RIGHT_POWER);
                    } else {
                        IntakeSystem.stop();
                    }
                })
                .setEnd((i) -> IntakeSystem.stop())
                .setFinish(() -> false);

        Mercurial.gamepad1().x()
                .whileTrue(new Lambda("HoldLeftGateRed")
                        .addRequirements(outtake)
                        .setExecute(() -> {
                            spinFlywheelToFar();
                            double rps = avgFlywheelRps();
                            boolean ready = Math.abs(FAR_TARGET_RPS - rps) <= READY_THRESHOLD_RPS;
                            OuttakeSystem.leftGate().setPosition(ready ? LEFT_GATE_OPEN : LEFT_GATE_CLOSED);
                            OuttakeSystem.rightGate().setPosition(RIGHT_GATE_CLOSED);
                        })
                        .setEnd((i) -> {
                            OuttakeSystem.leftGate().setPosition(LEFT_GATE_CLOSED);
                            OuttakeSystem.rightGate().setPosition(RIGHT_GATE_CLOSED);
                            stopFlywheel();
                        })
                        .setFinish(() -> false)
                )
                .onFalse(IntakeCommands.intakeLeftTimed(INTAKE_POWER, SORT_LEFT_POWER, POST_SHOT_FEED_S));

        Mercurial.gamepad1().b()
                .whileTrue(new Lambda("HoldRightGateRed")
                        .addRequirements(outtake)
                        .setExecute(() -> {
                            spinFlywheelToFar();
                            double rps = avgFlywheelRps();
                            boolean ready = Math.abs(FAR_TARGET_RPS - rps) <= READY_THRESHOLD_RPS;
                            OuttakeSystem.rightGate().setPosition(ready ? RIGHT_GATE_OPEN : RIGHT_GATE_CLOSED);
                            OuttakeSystem.leftGate().setPosition(LEFT_GATE_CLOSED);
                        })
                        .setEnd((i) -> {
                            OuttakeSystem.leftGate().setPosition(LEFT_GATE_CLOSED);
                            OuttakeSystem.rightGate().setPosition(RIGHT_GATE_CLOSED);
                            stopFlywheel();
                        })
                        .setFinish(() -> false)
                )
                .onFalse(IntakeCommands.intakeRightTimed(INTAKE_POWER, SORT_RIGHT_POWER, POST_SHOT_FEED_S));
    }

    @Override
    public void start() {
        driveCommand.schedule();
        intakeTeleopCommand.schedule();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        try {
            DriveSystem.stop();
        } catch (Exception ignored) {}

        IntakeSystem.stop();

        OuttakeSystem.leftGate().setPosition(LEFT_GATE_CLOSED);
        OuttakeSystem.rightGate().setPosition(RIGHT_GATE_CLOSED);
        stopFlywheel();

        try {
            VisionSystem.stop();
        } catch (Exception ignored) {}
    }
}