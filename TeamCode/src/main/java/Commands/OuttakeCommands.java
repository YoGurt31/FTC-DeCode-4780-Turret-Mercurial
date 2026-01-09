package Commands;

import androidx.annotation.NonNull;

import SubSystems.OuttakeSystem;
import dev.frozenmilk.mercurial.commands.Lambda;

public final class OuttakeCommands {

    private OuttakeCommands() {}

    public static final class Params {
        public double readyThresholdRps = 2.5;
        public double gateOpenSec = 0.50;
        public double betweenShotsSec = 0.25;
        public double maxSpinupSec = 3.0;

        public Params() {}
    }

    @NonNull
    public static Lambda stopFlywheel() {
        return new Lambda("Outtake.StopFlywheel")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setInit(OuttakeSystem::stopFlywheel)
                .setExecute(OuttakeSystem::stopFlywheel)
                .setFinish(() -> true)
                .setEnd(interrupted -> OuttakeSystem.stopFlywheel());
    }

    @NonNull
    public static Lambda spinFlywheelHold(double rps) {
        return new Lambda("Outtake.SpinHold(" + rps + ")")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setExecute(() -> OuttakeSystem.setFlywheelRps(rps))
                .setFinish(() -> false)
                .setEnd(interrupted -> OuttakeSystem.stopFlywheel());
    }

    @NonNull
    public static Lambda spinFlywheelUntilReady(double rps, @NonNull Params p) {
        final long[] startMs = new long[1];

        return new Lambda("Outtake.SpinUntilReady(" + rps + ")")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setInit(() -> {
                    startMs[0] = System.currentTimeMillis();
                    OuttakeSystem.setFlywheelRps(rps);
                })
                .setExecute(() -> OuttakeSystem.setFlywheelRps(rps))
                .setFinish(() -> {
                    boolean ready = OuttakeSystem.isFlywheelReady(rps, p.readyThresholdRps);
                    boolean timedOut = (p.maxSpinupSec > 0.0)
                            && (System.currentTimeMillis() - startMs[0]) >= (long) (p.maxSpinupSec * 1000.0);
                    return ready || timedOut;
                })
                .setEnd(interrupted -> {
                    // Intentionally do NOT stop flywheel here; caller decides.
                });
    }

    @NonNull
    public static Lambda leftGateHoldIfReady(double rps, @NonNull Params p) {
        return new Lambda("Outtake.LeftGateHoldIfReady")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setExecute(() -> {
                    if (OuttakeSystem.isFlywheelReady(rps, p.readyThresholdRps)) {
                        OuttakeSystem.openLeftGate();
                    } else {
                        OuttakeSystem.closeLeftGate();
                    }
                })
                .setFinish(() -> false)
                .setEnd(interrupted -> OuttakeSystem.closeLeftGate());
    }

    @NonNull
    public static Lambda rightGateHoldIfReady(double rps, @NonNull Params p) {
        return new Lambda("Outtake.RightGateHoldIfReady")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setExecute(() -> {
                    if (OuttakeSystem.isFlywheelReady(rps, p.readyThresholdRps)) {
                        OuttakeSystem.openRightGate();
                    } else {
                        OuttakeSystem.closeRightGate();
                    }
                })
                .setFinish(() -> false)
                .setEnd(interrupted -> OuttakeSystem.closeRightGate());
    }

    @NonNull
    public static Lambda fireLeftOnce(double rps, @NonNull Params p) {
        final int[] phase = new int[1];
        final long[] t0 = new long[1];

        return new Lambda("Outtake.FireLeftOnce")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setInit(() -> {
                    phase[0] = 0;
                    t0[0] = System.currentTimeMillis();
                    OuttakeSystem.setFlywheelRps(rps);
                    OuttakeSystem.closeLeftGate();
                })
                .setExecute(() -> {
                    OuttakeSystem.setFlywheelRps(rps);
                    long now = System.currentTimeMillis();

                    if (phase[0] == 0) {
                        if (OuttakeSystem.isFlywheelReady(rps, p.readyThresholdRps)) {
                            OuttakeSystem.openLeftGate();
                            phase[0] = 1;
                            t0[0] = now;
                        }
                    } else if (phase[0] == 1) {
                        if (now - t0[0] >= (long) (p.gateOpenSec * 1000.0)) {
                            OuttakeSystem.closeLeftGate();
                            phase[0] = 2;
                            t0[0] = now;
                        }
                    } else if (phase[0] == 2) {
                        if (now - t0[0] >= (long) (p.betweenShotsSec * 1000.0)) {
                            phase[0] = 3;
                        }
                    }
                })
                .setFinish(() -> phase[0] >= 3)
                .setEnd(interrupted -> OuttakeSystem.closeLeftGate());
    }

    @NonNull
    public static Lambda fireRightOnce(double rps, @NonNull Params p) {
        final int[] phase = new int[1];
        final long[] t0 = new long[1];

        return new Lambda("Outtake.FireRightOnce")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setInit(() -> {
                    phase[0] = 0;
                    t0[0] = System.currentTimeMillis();
                    OuttakeSystem.setFlywheelRps(rps);
                    OuttakeSystem.closeRightGate();
                })
                .setExecute(() -> {
                    OuttakeSystem.setFlywheelRps(rps);
                    long now = System.currentTimeMillis();

                    if (phase[0] == 0) {
                        if (OuttakeSystem.isFlywheelReady(rps, p.readyThresholdRps)) {
                            OuttakeSystem.openRightGate();
                            phase[0] = 1;
                            t0[0] = now;
                        }
                    } else if (phase[0] == 1) {
                        if (now - t0[0] >= (long) (p.gateOpenSec * 1000.0)) {
                            OuttakeSystem.closeRightGate();
                            phase[0] = 2;
                            t0[0] = now;
                        }
                    } else if (phase[0] == 2) {
                        if (now - t0[0] >= (long) (p.betweenShotsSec * 1000.0)) {
                            phase[0] = 3;
                        }
                    }
                })
                .setFinish(() -> phase[0] >= 3)
                .setEnd(interrupted -> OuttakeSystem.closeRightGate());
    }

    @NonNull
    public static Lambda closeGates() {
        return new Lambda("Outtake.CloseGates")
                .addRequirements(OuttakeSystem.INSTANCE)
                .setInit(() -> {
                    OuttakeSystem.closeLeftGate();
                    OuttakeSystem.closeRightGate();
                })
                .setExecute(() -> {
                    OuttakeSystem.closeLeftGate();
                    OuttakeSystem.closeRightGate();
                })
                .setFinish(() -> true);
    }
}