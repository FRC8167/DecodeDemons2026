package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
@Configurable
public class Gate extends SubsystemBase {

    public enum GateState {
        OPEN,
        CLOSED
    }

    private final ServoEx servo;
    private GateState state= GateState.CLOSED;

    private static final double CLOSED_POS = 0.31;//y
    private static final double OPEN_POS = 0.551;//x

    public Gate(ServoEx servo) {
        this.servo = servo;
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

    public void setGateState(Gate.GateState gateState)  {
        this.state = gateState;
        setGateState();
    }

    private void setGateState()  {
        switch(state) {
            case OPEN:
                open();
                break;
            case CLOSED:
                close();
                break;

        }
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
