package SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.List;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class VisionSystem implements Subsystem {

    public static final VisionSystem INSTANCE = new VisionSystem();
    private VisionSystem() {}

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}

    private Dependency dependency = Subsystem.DEFAULT_DEPENDENCY
            .and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency dependency) {
        this.dependency = dependency;
    }

    private final SubsystemObjectCell<Limelight3A> limeLight = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(Limelight3A.class, "limelight")
    );

    public static Limelight3A lL() { return INSTANCE.limeLight.get(); }

    public int desiredPipeline = 0;
    public final int RED = 1;
    public final int BLUE = 2;

    public int motifTagId = -1;
    public String motifPattern = "UNKNOWN";

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        // Safe startup defaults
        desiredPipeline = 0;
        resetMotif();

        try {
            lL().pipelineSwitch(0);
        } catch (Exception ignored) {}

        try {
            lL().start();
        } catch (Exception ignored) {}

        setDefaultCommand(idle());
    }

    @NonNull
    public static Lambda idle() {
        return new Lambda("vision.idle")
                .addRequirements(INSTANCE)
                .setExecute(() -> {
                    // No periodic work required by default.
                });
    }

    public static void setPipeline(int pipelineIndex) {
        INSTANCE.desiredPipeline = pipelineIndex;
        try {
            lL().pipelineSwitch(pipelineIndex);
        } catch (Exception ignored) {}
    }

    public static void setPollRateHz(int hz) {
        try {
            lL().setPollRateHz(hz);
        } catch (Exception ignored) {}
    }

    public static void resetMotif() {
        INSTANCE.motifTagId = -1;
        INSTANCE.motifPattern = "UNKNOWN";
    }

    public static boolean hasMotif() {
        return INSTANCE.motifTagId == 21 || INSTANCE.motifTagId == 22 || INSTANCE.motifTagId == 23;
    }

    public static void updateMotifOnce() {
        if (hasMotif()) return;

        LLResult result = getLatestResult();
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;

        int id = fiducials.get(0).getFiducialId();
        switch (id) {
            case 21:
                INSTANCE.motifTagId = 21;
                INSTANCE.motifPattern = "GPP";
                break;
            case 22:
                INSTANCE.motifTagId = 22;
                INSTANCE.motifPattern = "PGP";
                break;
            case 23:
                INSTANCE.motifTagId = 23;
                INSTANCE.motifPattern = "PPG";
                break;
            default:
                // ignore
                break;
        }
    }

    public static LLResult getLatestResult() {
        try {
            return lL().getLatestResult();
        } catch (Exception ignored) {
            return null;
        }
    }

    public static Integer getFirstFiducialId(LLResult result) {
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        return fiducials.get(0).getFiducialId();
    }

    public static LLResultTypes.FiducialResult getFiducialById(LLResult result, int id) {
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f != null && f.getFiducialId() == id) return f;
        }
        return null;
    }

    public static boolean seesFiducialId(int desiredId) {
        LLResult res = getLatestResult();
        Integer id = getFirstFiducialId(res);
        return id != null && id == desiredId;
    }

    public static Double getTxIfSeeing(int desiredId) {
        LLResult res = getLatestResult();
        if (res == null || !res.isValid()) return null;
        Integer id = getFirstFiducialId(res);
        if (id == null || id != desiredId) return null;
        return res.getTx();
    }

    public static void stop() {
        try {
            lL().stop();
        } catch (Exception ignored) {}
    }
}
