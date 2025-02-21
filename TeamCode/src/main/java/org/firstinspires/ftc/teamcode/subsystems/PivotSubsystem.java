package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.kWristName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.klElbowName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Pivot.krElbowName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class PivotSubsystem extends Subsystem {
    private final CachingServo wrist;
    private final CachingServo lElbow;
    private final CachingServo rElbow;

    public enum State {
        DRIVING(0.15, 0.3), //0 is down
        PREPARE_INTAKE(0, 0.08),
        PREPARE_INTAKE_HIGHER(0,0.09),
        PREPARE_INTAKE_HIGH(0,0.20),
        PREPARE_INTAKE_HIGHEST(0,0.30),
        INTAKE(0,0), //  0.63 wrist is level with floor
        PREPARE_PICKUP(0.93,0.325),
        PICKUP(0.93,0.825), //picks up from wall
        SUB_TO_DRIVING(1, 0.1),
        OUTTAKE_SPEC(0.275,0.325),
        OUTTAKE_BUCKET(1,0.5),
        HUMAN1(1,0.3),
        HUMAN2(1,0.8),
        PARK(0.8, 0.4),
        AUTO_SPEC(0.915, 0.5),
        AUTO_INTAKE(0.275,0.325),
        PASSIVE(0.55,0.12),
        START_SPEC_AUTO(0.55, 0.2),
        FINISH_PASSIVE(0.55,0.075),
        MANUAL(0,0);

        final double wristPos;
        final double elbowPos;

        State(double wristPos, double elbowPos) {
            this.wristPos = wristPos;
            this.elbowPos = elbowPos;
        }
    }

    private State state;
    private double manualWristPos, manualElbowPos;

    public PivotSubsystem(HardwareMap hardwareMap) {
        wrist = new CachingServo((Servo) hardwareMap.get(kWristName));
        lElbow = new CachingServo((Servo) hardwareMap.get(klElbowName));
        lElbow.setDirection(Servo.Direction.REVERSE);
        rElbow = new CachingServo((Servo) hardwareMap.get(krElbowName));

        setState(State.DRIVING);
        manualElbowPos = 0.3;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setWristPos(double pos) {
        wrist.setPosition(pos);
    }

    public void setElbowPos(double pos) {
        lElbow.setPosition(pos);
        rElbow.setPosition(pos);
    }

    public void incrementElbowUp() {
        if(lElbow.getPosition() < 0.83) {
            state = State.MANUAL;
            manualElbowPos = lElbow.getPosition() + 0.01;
        }
    }

    public void decrementElbowDown() {
        if(lElbow.getPosition() > 0.0) {
            state = State.MANUAL;
            manualElbowPos = lElbow.getPosition() - 0.01;
        }
    }

    public void incrementWristUp() {
        if(wrist.getPosition() < 1.0) {
            state = State.MANUAL;
            manualWristPos = wrist.getPosition() + 0.01;
        }
    }

    public void decrementWristDown() {
        if(wrist.getPosition() > 0.0) {
            state = State.MANUAL;
            manualWristPos = wrist.getPosition() - 0.01;
        }
    }

    public void processState() {
        if (this.state != State.MANUAL) {
            setElbowPos(this.state.elbowPos);
            setWristPos(this.state.wristPos);
        }
        else {
            setElbowPos(manualElbowPos);
            setWristPos(manualWristPos);
        }
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();

        telemetry.addData("Pivot State: ", state);
    }
}
