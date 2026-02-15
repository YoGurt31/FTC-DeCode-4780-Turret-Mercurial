package SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.Constants;

public class Release {
    public static final Release INSTANCE = new Release();

    private Release() {
    }

    private Servo release;

    @SuppressWarnings("unused")
    private Telemetry telemetry;

    private boolean isOpen = false;
    private long openedAtMs = 0;

    private static final long AUTO_CLOSE_MS = 10000;

    public void init(HardwareMap hw, Telemetry telem) {
        this.telemetry = telem;

        release = hw.get(Servo.class, Constants.Releases.artifactRelease);

        close();
        isOpen = false;
        openedAtMs = 0;
    }

    public void update() {
        if (!isOpen) return;

        long elapsed = System.currentTimeMillis() - openedAtMs;
        if (elapsed >= AUTO_CLOSE_MS) {
            close();
        }
    }

    public void open() {
        if (release != null) release.setPosition(Constants.Releases.RELEASE);
        isOpen = true;
        openedAtMs = System.currentTimeMillis();
    }

    public void close() {
        if (release != null) release.setPosition(Constants.Releases.HOLD);
        isOpen = false;
        openedAtMs = 0;
    }

    public void toggle() {
        if (release == null) return;
        if (isOpen) close();
        else open();
    }

    public boolean isGateOpen() {
        return release != null && isOpen;
    }

    public boolean isShooting() {
        return isOpen;
    }
}
