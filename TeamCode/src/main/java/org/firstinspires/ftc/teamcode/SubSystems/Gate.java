package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Gate extends SubsystemBase {

    public enum GateState {
        OPEN,
        CLOSED
    }

    private final ServoEx servo;
    private GateState state = GateState.CLOSED;

    private static final double OPEN_POS = 0.5;
    private static final double CLOSED_POS = 0.1;

    public Gate(ServoEx servo) {
        this.servo = servo;
        close();
    }

    /* ===== Public API ===== */

    public void open() {
        servo.set(OPEN_POS);
        state = GateState.OPEN;
    }

    public void close() {
        servo.set(CLOSED_POS);
        state = GateState.CLOSED;
    }

    public void toggle() {
        if (state == GateState.OPEN) {
            close();
        } else {
            open();
        }
    }

    public GateState getState() {
        return state;
    }

    public boolean isOpen() {
        return state == GateState.OPEN;
    }
}
