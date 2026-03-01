package Util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public final class Actions {

    private Actions() {}

    public static Action instant(Runnable r) {
        return new InstantAction(r);
    }

    public static Action waitSeconds(double seconds) {
        return new WaitAction(seconds);
    }

    public static Action runFor(double seconds, Runnable onLoop) {
        return new RunForAction(seconds, onLoop);
    }

    public static Action until(Condition cond, double timeoutSec, Runnable onLoop) {
        return new UntilAction(cond, timeoutSec, onLoop);
    }

    public static Action sequence(Action... actions) {
        return new SequenceAction(actions);
    }

    public static Action parallel(Action... actions) {
        return new ParallelAction(actions);
    }

    public interface Condition {
        boolean get();
    }

    private static final class InstantAction implements Action {
        private final Runnable r;
        private boolean ran;

        InstantAction(Runnable r) {
            this.r = r;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!ran) {
                ran = true;
                if (r != null) r.run();
            }
            return false;
        }
    }

    private static final class WaitAction implements Action {
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

    private static final class RunForAction implements Action {
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

    private static final class UntilAction implements Action {
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
            boolean timedOut = timeoutSec > 0.0 && timer.seconds() >= timeoutSec;

            return !(done || timedOut);
        }
    }

    private static final class SequenceAction implements Action {
        private final Action[] actions;
        private int index;

        SequenceAction(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actions == null || actions.length == 0) return false;

            while (index < actions.length) {
                Action current = actions[index];
                if (current == null) {
                    index++;
                    continue;
                }

                boolean keepRunning = current.run(packet);
                if (keepRunning) return true;

                index++;
            }
            return false;
        }
    }

    private static final class ParallelAction implements Action {
        private final Action[] actions;

        ParallelAction(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (actions == null || actions.length == 0) return false;

            boolean anyRunning = false;

            for (Action a : actions) {
                if (a != null && a.run(packet)) {
                    anyRunning = true;
                }
            }

            return anyRunning;
        }
    }
}
