package SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class OuttakeSystem implements Subsystem {

    public static final OuttakeSystem INSTANCE = new OuttakeSystem();
    private OuttakeSystem() {}

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

    private final SubsystemObjectCell<DcMotorEx> flyWheel1 = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fW1")
    );
    private final SubsystemObjectCell<DcMotorEx> flyWheel2 = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fW2")
    );

    private final SubsystemObjectCell<Servo> leftRelease = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "lR")
    );
    private final SubsystemObjectCell<Servo> rightRelease = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "rR")
    );

    public static DcMotorEx fw1() { return INSTANCE.flyWheel1.get(); }
    public static DcMotorEx fw2() { return INSTANCE.flyWheel2.get(); }
    public static Servo leftGate() { return INSTANCE.leftRelease.get(); }
    public static Servo rightGate() { return INSTANCE.rightRelease.get(); }

    public double targetRps = 0.0;

    public final double farTargetRps = 52.5;
    public final double closeTargetRps = 46.5;

    public final double ticksPerRev = 28.0;

    public final double artifactHoldRight = 0.5;
    public final double artifactHoldLeft = 0.0;
    public final double artifactReleaseRight = 1.0;
    public final double artifactReleaseLeft = 0.5;

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        double F = 17.75;
        double P = 2.5 * F;
        double I = 0.0;
        double D = 0.0;

        fw1().setDirection(DcMotorEx.Direction.FORWARD);
        fw1().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fw1().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fw1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fw1().setVelocityPIDFCoefficients(P, I, D, F);

        fw2().setDirection(DcMotorEx.Direction.REVERSE);
        fw2().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fw2().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fw2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fw2().setVelocityPIDFCoefficients(P, I, D, F);

        closeLeftGate();
        closeRightGate();
        stopFlywheel();

        setDefaultCommand(idle());
    }

    @NonNull
    public static Lambda idle() {
        return new Lambda("outtake.idle")
                .addRequirements(INSTANCE)
                .setExecute(() -> {
                    // Keep gates in their last commanded positions; just ensure motors are not accidentally powered.
                    if (INSTANCE.targetRps <= 0.0) {
                        stopFlywheel();
                    }
                });
    }

    public static void setFlywheelRps(double rps) {
        INSTANCE.targetRps = rps;
        double targetTicksPerSec = rps * INSTANCE.ticksPerRev;
        fw1().setVelocity(targetTicksPerSec);
        fw2().setVelocity(targetTicksPerSec);
    }

    public static void stopFlywheel() {
        INSTANCE.targetRps = 0.0;
        fw1().setPower(0.0);
        fw2().setPower(0.0);
    }

    public static double flywheelRps1() {
        return Math.abs(fw1().getVelocity()) / INSTANCE.ticksPerRev;
    }

    public static double flywheelRps2() {
        return Math.abs(fw2().getVelocity()) / INSTANCE.ticksPerRev;
    }

    public static double flywheelRpsAvg() {
        return (flywheelRps1() + flywheelRps2()) / 2.0;
    }

    public static boolean isFlywheelReady(double targetRps, double thresholdRps) {
        return flywheelRpsAvg() >= (targetRps - Math.abs(thresholdRps));
    }

    public static void openLeftGate() {
        leftGate().setPosition(INSTANCE.artifactReleaseLeft);
    }

    public static void closeLeftGate() {
        leftGate().setPosition(INSTANCE.artifactHoldLeft);
    }

    public static void openRightGate() {
        rightGate().setPosition(INSTANCE.artifactReleaseRight);
    }

    public static void closeRightGate() {
        rightGate().setPosition(INSTANCE.artifactHoldRight);
    }
}