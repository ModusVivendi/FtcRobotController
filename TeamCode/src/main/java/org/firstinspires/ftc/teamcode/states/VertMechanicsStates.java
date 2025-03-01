package org.firstinspires.ftc.teamcode.states;

public class VertMechanicsStates {

    public enum VertSlideState {
        IDLE,
        MANUAL_CONTROL,
        MOVING_TO_POSITION,
        HOLDING_POSITION,
        ERROR
    }

    public enum VertGripperState {
        INIT,
        IDLE,
        OPENING,
        CLOSING,
        ERROR
    }

    public enum VertClawState {
        INIT,
        IDLE,
        OPEN,
        CLOSE,
        HOME,
        TRANSFER,
        ERROR
    }

    public enum VertRotationState {
        INIT,
        LEFT_LIMIT,
        CENTER,
        RIGHT_LIMIT,
        IDLE,
        ROTATING_LEFT,
        ROTATING_RIGHT,
        TRANSFER,
        ERROR
    }
}
