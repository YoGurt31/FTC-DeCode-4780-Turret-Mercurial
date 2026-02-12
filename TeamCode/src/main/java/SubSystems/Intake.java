package SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Intake {
    public static final Intake INSTANCE = new Intake();

    private Intake() {
    }

    public enum Mode {
        IDLE,
        INTAKE,
        EJECT
    }

    private DcMotorEx roller;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private Mode mode = Mode.IDLE;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        roller = hw.get(DcMotorEx.class, Constants.Intake.rollerIntake);
        roller.setDirection(DcMotorSimple.Direction.REVERSE);

        roller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        roller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setMode(Mode.IDLE);
        apply();
    }

    public void setMode(Mode newMode) {
        this.mode = newMode == null ? Mode.IDLE : newMode;
    }

    public Mode getMode() {
        return mode;
    }

    public void apply() {
        if (roller == null) return;

        switch (mode) {
            case INTAKE:
                roller.setPower(Constants.Intake.INTAKE_POWER);
                break;

            case EJECT:
                roller.setPower(Constants.Intake.OUTTAKE_POWER);
                break;

            case IDLE:
            default:
                roller.setPower(0);
                break;
        }
    }

    public void stop() {
        setMode(Mode.IDLE);
        apply();
    }

    public String getSortingStatus() {
        switch (mode) {
            case INTAKE:
                return "Intake";
            case EJECT:
                return "Eject";
            case IDLE:
            default:
                return "Idle";
        }
    }

    public boolean isActive() {
        return mode != Mode.IDLE;
    }
}
