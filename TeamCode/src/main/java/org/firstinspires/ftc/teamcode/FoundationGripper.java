package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGripper {
    public enum GripperState {
        FOUNDATION_OPEN,
        FOUNDATION_CLOSE,
        BLOCK_OPEN,
        BLOCK_CLOSE;
    }

    private final double L_BLOCK_CLOSE = 0.87, L_BLOCK_OPEN = 0.99, L_FOUNDATION_OPEN = 0.65, L_FOUNDATION_CLOSE = 0.49;
    private final double R_BLOCK_CLOSE = 0.36, R_BLOCK_OPEN = 0.23, R_FOUNDATION_OPEN = 0.625, R_FOUNDATION_CLOSE = 0.75;


    private GripperState state;
        public GripperState getState() {return state;}
        public void setState(GripperState newState) {state = newState;}

    private Servo left, right;

    public FoundationGripper(Servo l, Servo r) {
        left = l;
        right = r;
    }

    public void update(boolean increment, boolean decrement) {
        if(increment && !decrement) {
            switch (state) {
                case BLOCK_CLOSE:
                    state = GripperState.BLOCK_OPEN;
                    break;
                case FOUNDATION_OPEN:
                    state = GripperState.BLOCK_CLOSE;
                    break;
                case FOUNDATION_CLOSE:
                    state = GripperState.FOUNDATION_OPEN;
                    break;
            }
        }
        if(decrement && !increment) {
            switch (state) {
                case BLOCK_OPEN:
                    state = GripperState.BLOCK_CLOSE;
                    break;
                case BLOCK_CLOSE:
                    state = GripperState.FOUNDATION_OPEN;
                    break;
                case FOUNDATION_OPEN:
                    state = GripperState.FOUNDATION_CLOSE;
                    break;
            }
        }

        switch (state) {
            case FOUNDATION_CLOSE:
                left.setPosition(L_FOUNDATION_CLOSE);
                right.setPosition(R_FOUNDATION_CLOSE);
                break;
            case FOUNDATION_OPEN:
                left.setPosition(L_FOUNDATION_OPEN);
                right.setPosition(R_FOUNDATION_OPEN);
                break;
            case BLOCK_CLOSE:
                left.setPosition(L_BLOCK_CLOSE);
                right.setPosition(R_BLOCK_CLOSE);
                break;
            case BLOCK_OPEN:
                left.setPosition(L_BLOCK_OPEN);
                right.setPosition(R_BLOCK_OPEN);
                break;
        }
    }

    public void update(GripperState newState) {
        state = newState;

        switch (state) {
            case FOUNDATION_CLOSE:
                left.setPosition(L_FOUNDATION_CLOSE);
                right.setPosition(R_FOUNDATION_CLOSE);
                break;
            case FOUNDATION_OPEN:
                left.setPosition(L_FOUNDATION_OPEN);
                right.setPosition(R_FOUNDATION_OPEN);
                break;
            case BLOCK_CLOSE:
                left.setPosition(L_BLOCK_CLOSE);
                right.setPosition(R_BLOCK_CLOSE);
                break;
            case BLOCK_OPEN:
                left.setPosition(L_BLOCK_OPEN);
                right.setPosition(R_BLOCK_OPEN);
                break;
        }
    }

    public void update() {
        update(state);
    }
}
