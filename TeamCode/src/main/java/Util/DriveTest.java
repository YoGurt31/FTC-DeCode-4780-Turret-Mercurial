package Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Drive Test", group = "Test")
public final class DriveTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        DcMotor fL = hardwareMap.get(DcMotor.class, "fL");
        DcMotor fR = hardwareMap.get(DcMotor.class, "fR");
        DcMotor bL = hardwareMap.get(DcMotor.class, "bL");
        DcMotor bR = hardwareMap.get(DcMotor.class, "bR");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        telemetry.addLine("Drive Test Ready. DPad Drives/Rotates. Y/B/X/A Run fL/fR/bL/bR.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            pinpoint.update();

            boolean dpad = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;

            boolean face = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            final double base = 0.5;

            double fLpower = 0.0, fRpower = 0.0, bLpower = 0.0, bRpower = 0.0;
            String mode = "IDLE";

            if (face) {
                if (gamepad1.y) {
                    fLpower = base;
                    mode = "MOTOR: fL (Y)";
                } else if (gamepad1.b) {
                    fRpower = base;
                    mode = "MOTOR: fR (B)";
                } else if (gamepad1.x) {
                    bLpower = base;
                    mode = "MOTOR: bL (X)";
                } else {
                    bRpower = base;
                    mode = "MOTOR: bR (A)";
                }
            } else if (dpad) {
                if (gamepad1.dpad_up) {
                    fLpower = base;
                    bLpower = base;
                    fRpower = base;
                    bRpower = base;
                    mode = "DPAD: FORWARD";
                } else if (gamepad1.dpad_down) {
                    fLpower = -base;
                    bLpower = -base;
                    fRpower = -base;
                    bRpower = -base;
                    mode = "DPAD: BACK";
                } else if (gamepad1.dpad_left) {
                    fLpower = -base;
                    bLpower = -base;
                    fRpower = base;
                    bRpower = base;
                    mode = "DPAD: ROTATE LEFT";
                } else if (gamepad1.dpad_right) {
                    fLpower = base;
                    bLpower = base;
                    fRpower = -base;
                    bRpower = -base;
                    mode = "DPAD: ROTATE RIGHT";
                }
            }

            fL.setPower(fLpower);
            fR.setPower(fRpower);
            bL.setPower(bLpower);
            bR.setPower(bRpower);

            telemetry.addData("Mode", mode);
            telemetry.addLine();
            telemetry.addData("X (ticks)", pinpoint.getEncoderX());
            telemetry.addData("Y (ticks)", pinpoint.getEncoderY());
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("fL", fLpower);
            telemetry.addData("fR", fRpower);
            telemetry.addData("bL", bLpower);
            telemetry.addData("bR", bRpower);
            telemetry.update();

            idle();
        }
    }
}
