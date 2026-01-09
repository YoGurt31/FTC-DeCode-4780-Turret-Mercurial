package SubSystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

public class DriveSystem implements Subsystem {

    public static final DriveSystem INSTANCE = new DriveSystem();
    private DriveSystem() {}

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

    private final SubsystemObjectCell<DcMotorEx> frontLeft = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fL"));
    private final SubsystemObjectCell<DcMotorEx> frontRight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "fR"));
    private final SubsystemObjectCell<DcMotorEx> backLeft = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "bL"));
    private final SubsystemObjectCell<DcMotorEx> backRight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "bR"));

    private final SubsystemObjectCell<GoBildaPinpointDriver> pinPoint = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));

    private final SubsystemObjectCell<Servo> gearShift = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "gS"));
    private final SubsystemObjectCell<Servo> elevatorLeft = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "eL"));
    private final SubsystemObjectCell<Servo> elevatorRight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Servo.class, "eR"));

    public static DcMotorEx frontLeft() { return INSTANCE.frontLeft.get(); }
    public static DcMotorEx frontRight() { return INSTANCE.frontRight.get(); }
    public static DcMotorEx backLeft() { return INSTANCE.backLeft.get(); }
    public static DcMotorEx backRight() { return INSTANCE.backRight.get(); }

    public static GoBildaPinpointDriver pinPoint() { return INSTANCE.pinPoint.get(); }

    public static Servo gearShift() { return INSTANCE.gearShift.get(); }
    public static Servo elevatorLeft() { return INSTANCE.elevatorLeft.get(); }
    public static Servo elevatorRight() { return INSTANCE.elevatorRight.get(); }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        frontLeft().setDirection(DcMotorEx.Direction.FORWARD);
        backLeft().setDirection(DcMotorEx.Direction.FORWARD);
        frontRight().setDirection(DcMotorEx.Direction.REVERSE);
        backRight().setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight().setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pinPoint().setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinPoint().setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint().setOffsets(-176, -66, DistanceUnit.MM);
        pinPoint().resetPosAndIMU();
        pinPoint().setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        gearShift().setDirection(Servo.Direction.FORWARD);
        elevatorLeft().setDirection(Servo.Direction.FORWARD);
        elevatorRight().setDirection(Servo.Direction.FORWARD);

        gearShift().setPosition(0.5);
        elevatorLeft().setPosition(0.5);
        elevatorRight().setPosition(0.5);

        setDefaultCommand(idle());
    }

    @NonNull
    public static Lambda idle() {
        return new Lambda("drive.idle")
                .addRequirements(INSTANCE)
                .setExecute(() -> tankDrive(0.0, 0.0));
    }

    public static void tankDrive(double drive, double rotate) {
        double leftPower = drive + rotate;
        double rightPower = drive - rotate;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        frontLeft().setPower(leftPower);
        frontRight().setPower(rightPower);
        backLeft().setPower(leftPower);
        backRight().setPower(rightPower);
    }

    public static void stop() {
        tankDrive(0.0, 0.0);
    }

    public static void updatePinpoint() {
        pinPoint().update();
    }

    public static Pose2D pose() {
        return pinPoint().getPosition();
    }

    public static double headingDeg() {
        return pose().getHeading(AngleUnit.DEGREES);
    }

    public static double wrapDeg(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
}