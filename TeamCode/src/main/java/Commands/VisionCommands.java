package Commands;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;

import SubSystems.VisionSystem;
import dev.frozenmilk.mercurial.commands.Lambda;

public final class VisionCommands {

    private VisionCommands() {}

    public static final class Params {
        public double motifReadTimeoutSec = 3.0;
        public int pollRateHz = 30;

        public Params() {}
    }

    @NonNull
    public static Lambda setPipeline(int pipelineIndex) {
        return new Lambda("Vision.SetPipeline(" + pipelineIndex + ")")
                .addRequirements(VisionSystem.INSTANCE)
                .setInit(() -> VisionSystem.setPipeline(pipelineIndex))
                .setExecute(() -> VisionSystem.setPipeline(pipelineIndex))
                .setFinish(() -> true);
    }

    @NonNull
    public static Lambda setPollRate(int hz) {
        return new Lambda("Vision.SetPollRate(" + hz + ")")
                .addRequirements(VisionSystem.INSTANCE)
                .setInit(() -> VisionSystem.setPollRateHz(hz))
                .setExecute(() -> VisionSystem.setPollRateHz(hz))
                .setFinish(() -> true);
    }

    @NonNull
    public static Lambda resetMotif() {
        return new Lambda("Vision.ResetMotif")
                .addRequirements(VisionSystem.INSTANCE)
                .setInit(VisionSystem::resetMotif)
                .setExecute(VisionSystem::resetMotif)
                .setFinish(() -> true);
    }

    @NonNull
    public static Lambda readMotifOnce() {
        return new Lambda("Vision.ReadMotifOnce")
                .addRequirements(VisionSystem.INSTANCE)
                .setExecute(VisionSystem::updateMotifOnce)
                .setFinish(() -> true);
    }

    @NonNull
    public static Lambda readMotifUntilFound(@NonNull Params p) {
        final long[] startMs = new long[1];

        return new Lambda("Vision.ReadMotifUntilFound")
                .addRequirements(VisionSystem.INSTANCE)
                .setInit(() -> {
                    startMs[0] = System.currentTimeMillis();
                    VisionSystem.setPollRateHz(p.pollRateHz);
                    VisionSystem.setPipeline(0);
                    VisionSystem.resetMotif();
                })
                .setExecute(VisionSystem::updateMotifOnce)
                .setFinish(() -> {
                    boolean found = VisionSystem.hasMotif();
                    boolean timedOut = (p.motifReadTimeoutSec > 0.0)
                            && (System.currentTimeMillis() - startMs[0]) >= (long) (p.motifReadTimeoutSec * 1000.0);
                    return found || timedOut;
                });
    }

    @NonNull
    public static Lambda waitForFiducialInView(int desiredId, double timeoutSec) {
        final long[] startMs = new long[1];

        return new Lambda("Vision.WaitForTag(" + desiredId + ")")
                .addRequirements(VisionSystem.INSTANCE)
                .setInit(() -> startMs[0] = System.currentTimeMillis())
                .setExecute(() -> {
                    LLResult ignored = VisionSystem.getLatestResult();
                })
                .setFinish(() -> {
                    boolean seen = VisionSystem.seesFiducialId(desiredId);
                    boolean timedOut = (timeoutSec > 0.0)
                            && (System.currentTimeMillis() - startMs[0]) >= (long) (timeoutSec * 1000.0);
                    return seen || timedOut;
                });
    }
}