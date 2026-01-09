package Commands;

import androidx.annotation.NonNull;

import SubSystems.IntakeSystem;
import dev.frozenmilk.mercurial.commands.Lambda;

public final class IntakeCommands {

    private IntakeCommands() {}

    @NonNull
    public static Lambda intakeLeftHold(double rollerPower, double sorterPower) {
        return new Lambda("IntakeLeftHold")
                .addRequirements(IntakeSystem.INSTANCE)
                .setExecute(() -> IntakeSystem.intakeLeft(rollerPower, sorterPower))
                .setFinish(() -> false)
                .setEnd(interrupted -> IntakeSystem.stop());
    }

    @NonNull
    public static Lambda intakeRightHold(double rollerPower, double sorterPower) {
        return new Lambda("IntakeRightHold")
                .addRequirements(IntakeSystem.INSTANCE)
                .setExecute(() -> IntakeSystem.intakeRight(rollerPower, sorterPower))
                .setFinish(() -> false)
                .setEnd(interrupted -> IntakeSystem.stop());
    }

    @NonNull
    public static Lambda stop() {
        return new Lambda("IntakeStop")
                .addRequirements(IntakeSystem.INSTANCE)
                .setInit(IntakeSystem::stop)
                .setExecute(IntakeSystem::stop)
                .setFinish(() -> true)
                .setEnd(interrupted -> IntakeSystem.stop());
    }

    @NonNull
    public static Lambda intakeLeftTimed(double rollerPower, double sorterPower, double seconds) {
        final long[] endMs = new long[1];

        return new Lambda("IntakeLeftTimed(" + seconds + ")")
                .addRequirements(IntakeSystem.INSTANCE)
                .setInit(() -> {
                    IntakeSystem.intakeLeft(rollerPower, sorterPower);
                    endMs[0] = System.currentTimeMillis() + (long) (seconds * 1000.0);
                })
                .setExecute(() -> IntakeSystem.intakeLeft(rollerPower, sorterPower))
                .setFinish(() -> System.currentTimeMillis() >= endMs[0])
                .setEnd(interrupted -> IntakeSystem.stop());
    }

    @NonNull
    public static Lambda intakeRightTimed(double rollerPower, double sorterPower, double seconds) {
        final long[] endMs = new long[1];

        return new Lambda("IntakeRightTimed(" + seconds + ")")
                .addRequirements(IntakeSystem.INSTANCE)
                .setInit(() -> {
                    IntakeSystem.intakeRight(rollerPower, sorterPower);
                    endMs[0] = System.currentTimeMillis() + (long) (seconds * 1000.0);
                })
                .setExecute(() -> IntakeSystem.intakeRight(rollerPower, sorterPower))
                .setFinish(() -> System.currentTimeMillis() >= endMs[0])
                .setEnd(interrupted -> IntakeSystem.stop());
    }
}
