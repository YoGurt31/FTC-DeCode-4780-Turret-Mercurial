package SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

public class IntakeSystem implements Subsystem {

    public static final IntakeSystem INSTANCE = new IntakeSystem();
    private IntakeSystem() {}

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

    private final SubsystemObjectCell<DcMotorEx> rollerIntake = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "rI")
    );

    private final SubsystemObjectCell<DcMotorEx> sorterIntake = subsystemCell(() ->
            FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "sI")
    );

    public static DcMotorEx roller() { return INSTANCE.rollerIntake.get(); }
    public static DcMotorEx sorter() { return INSTANCE.sorterIntake.get(); }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        roller().setDirection(DcMotorEx.Direction.FORWARD);
        roller().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roller().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roller().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sorter().setDirection(DcMotorEx.Direction.FORWARD);
        sorter().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorter().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sorter().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setDefaultCommand(idle());
    }

    @NonNull
    public static Lambda idle() {
        return new Lambda("intake.idle")
                .addRequirements(INSTANCE)
                .setExecute(IntakeSystem::stop);
    }

    public static void intakeLeft(double rollerPower, double sorterPower) {
        roller().setPower(rollerPower);
        sorter().setPower(Math.abs(sorterPower));
    }

    public static void intakeRight(double rollerPower, double sorterPower) {
        roller().setPower(rollerPower);
        sorter().setPower(-Math.abs(sorterPower));
    }

    public static void stop() {
        roller().setPower(0.0);
        sorter().setPower(0.0);
    }
}
