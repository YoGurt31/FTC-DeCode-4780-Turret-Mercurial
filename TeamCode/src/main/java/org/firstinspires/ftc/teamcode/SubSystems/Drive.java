package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.Util.Constants;

public class Drive {
    public static final Drive INSTANCE = new Drive();

    private Drive() {
    }

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private GoBildaPinpointDriver pinpoint;

    private IMU imu;

    private Iterable<VoltageSensor> voltageSensor;

    private boolean resetPinPointOnInit = true;
    public void setResetPinPointOnInit(boolean enabled) {
        resetPinPointOnInit = enabled;
    }

    public void setPose(double xIn, double yIn, double headingDeg) {
        if (pinpoint == null) return;
        try { pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.DEGREES, headingDeg)); } catch (Exception ignored) { }
    }


    @SuppressWarnings("unused")
    private Telemetry telemetry;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        voltageSensor = hw.voltageSensor;

        try {
            imu = hw.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            imu.initialize(new IMU.Parameters(orientation));
            imu.resetYaw();
        } catch (Exception ignored) {
            imu = null;
        }

        frontLeft = hw.get(DcMotorEx.class, Constants.Drive.FRONT_LEFT);
        frontRight = hw.get(DcMotorEx.class, Constants.Drive.FRONT_RIGHT);
        backLeft = hw.get(DcMotorEx.class, Constants.Drive.BACK_LEFT);
        backRight = hw.get(DcMotorEx.class, Constants.Drive.BACK_RIGHT);

        frontLeft.setDirection(Constants.Drive.LEFT_DIR);
        backLeft.setDirection(Constants.Drive.LEFT_DIR);
        frontRight.setDirection(Constants.Drive.RIGHT_DIR);
        backRight.setDirection(Constants.Drive.RIGHT_DIR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hw.get(GoBildaPinpointDriver.class, Constants.PinPoint.PINPOINT);

        pinpoint.setEncoderDirections(
                Constants.PinPoint.X_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                Constants.PinPoint.Y_REVERSED
                        ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(Constants.PinPoint.X_OFFSET_MM, Constants.PinPoint.Y_OFFSET_MM, DistanceUnit.MM);

        if (resetPinPointOnInit) {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    Constants.PinPoint.START_X_IN,
                    Constants.PinPoint.START_Y_IN,
                    AngleUnit.DEGREES,
                    Constants.PinPoint.START_HEADING_DEG
            ));
        }
    }

    public void updateOdometry() {
        if (pinpoint == null) return;
        try {
            pinpoint.update();
        } catch (Exception ignored) { }
    }

    public double getVx() { return pinpoint.getVelX(DistanceUnit.INCH); }
    public double getVy() { return pinpoint.getVelY(DistanceUnit.INCH); }

    public void drive(double driveInput, double turnInput) {
        double drive = driveInput;
        double turn = turnInput;

        drive = Range.clip(drive, -1.0, 1.0);

        double left = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        setLeftPower(left);
        setRightPower(right);
    }

    public void setLeftPower(double power) {
        if (frontLeft == null) return;
        frontLeft.setPower(power);
        backLeft.setPower(power);
    }

    public void setRightPower(double power) {
        if (frontRight == null) return;
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void stop() {
        setLeftPower(0);
        setRightPower(0);
    }

    public double getX() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    // FIXME
    public void relocalizePose(Pose3D llPose) {
        if (llPose == null) return;
        if (pinpoint == null) return;

        double robotXIn = llPose.getPosition().x * Constants.Relocalize.METERS_TO_IN;
        double robotYIn = llPose.getPosition().y * Constants.Relocalize.METERS_TO_IN;
        double yawDeg = llPose.getOrientation().getYaw(AngleUnit.DEGREES);

//        double currentH = getHeading();
//        double altYaw = Constants.wrapDeg(yawDeg + 180.0);
//        double err1 = Math.abs(Constants.wrapDeg(yawDeg - currentH));
//        double err2 = Math.abs(Constants.wrapDeg(altYaw - currentH));
//        if (err2 < err1) yawDeg = altYaw;

        try { pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotXIn, robotYIn, AngleUnit.DEGREES, yawDeg)); } catch (Exception ignored) { }
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        if (voltageSensor == null) return 0.0;
        for (VoltageSensor sensor : voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        if (!Double.isFinite(result)) return 12.0;
        return result;
    }
}