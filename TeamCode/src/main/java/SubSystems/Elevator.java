package SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Elevator {
    public static final Elevator INSTANCE = new Elevator();

    private Elevator() {
    }

    private Servo gearShift, elevatorLeft, elevatorRight;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private boolean rising = false;
    private long riseStartMs = 0;
    private long riseDurationMs = 0;
    private double risePower = 0.0;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        gearShift = hw.get(Servo.class, Constants.Elevator.gearShift);
        elevatorLeft = hw.get(Servo.class, Constants.Elevator.elevatorLeft);
        elevatorRight = hw.get(Servo.class, Constants.Elevator.elevatorRight);

        setDefaultPositions();
    }

    public void setDefaultPositions() {
        if (gearShift == null) return;
        gearShift.setPosition(Constants.Elevator.GEAR_SHIFT_DEFAULT);
        elevatorLeft.setPosition(Constants.Elevator.ELEVATOR_LEFT_DEFAULT);
        elevatorRight.setPosition(Constants.Elevator.ELEVATOR_RIGHT_DEFAULT);
    }

    public void applyPreset() {
        if (gearShift == null) return;
        gearShift.setPosition(Constants.Elevator.GEAR_SHIFT_PRESET);
        elevatorLeft.setPosition(Constants.Elevator.ELEVATOR_LEFT_PRESET);
        elevatorRight.setPosition(Constants.Elevator.ELEVATOR_RIGHT_PRESET);
    }

    public void startRise() {
        rising = true;
        riseStartMs = System.currentTimeMillis();
        riseDurationMs = 5000;
        risePower = -1.0;

        Drive.INSTANCE.setLeftPower(0);
    }

    public void updateRise() {
        if (!rising) return;

        long elapsed = System.currentTimeMillis() - riseStartMs;
        if (elapsed >= riseDurationMs) {
            rising = false;
            Drive.INSTANCE.stop();
            return;
        }

        Drive.INSTANCE.setRightPower(risePower);
    }

    public boolean isRising() {
        return rising;
    }
}
