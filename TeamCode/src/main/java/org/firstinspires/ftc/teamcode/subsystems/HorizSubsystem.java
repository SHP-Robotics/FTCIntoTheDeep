package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kLeftHorizSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRailName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Horiz.kRightHorizSlideName;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class HorizSubsystem extends Subsystem {
    // todo: consider renaming Horiz to horizontal in all files, OR don't if you don't want to
    // it is pretty inconsistent between and within subsystems
    private final CachingServo lHoriz;
    private final CachingServo rHoriz;
    private final CachingServo rail;

    public enum State {
        INTAKE_WALL(0.5, 0),
        SPECIMEN_DEPOSIT(0.75, 0),
        WALL_PICKUP_AUTO(1,0.2),
        DRIVING(0, 0), //1, 0.725 is all in
        BLOCK_IN_BOT(0,0.5),
        INTAKING_EXTENDED(1,1), // 0.55 rail max out, 0 slide max out
        PREP_AUTO_INTAKE(0.5,0.05),
        SUB_AUTO_INTAKE(0.75,0.75),
        PARK(0.725,0),
        PASSIVE(0,0.6),
        MANUAL(0.7,0);

        final double railPos;
        final double slidePos;

        State(double railPos, double slidePos) {
            this.railPos = railPos;
            this.slidePos = slidePos;
        }
    }

    private State state;
    public State prevState;
    private double manualHorizPos, manualRailPos;

    public HorizSubsystem(HardwareMap hardwareMap) {
        manualHorizPos = 0.0;
        manualRailPos = 0.0;

        lHoriz = new CachingServo((Servo) hardwareMap.get(kLeftHorizSlideName));
        lHoriz.setDirection(Servo.Direction.REVERSE);
        lHoriz.scaleRange(0.325, 0.9);

        rHoriz = new CachingServo((Servo) hardwareMap.get(kRightHorizSlideName));

        rHoriz.setDirection(Servo.Direction.FORWARD);
        lHoriz.scaleRange(0.325, 0.9);

        rail = new CachingServo((Servo) hardwareMap.get(kRailName));
        rail.setDirection(Servo.Direction.FORWARD);
        rail.scaleRange(0, 0.45); //0 in, 0.45 out

        setState(State.DRIVING);
        prevState = state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void incrementHorizSlide() {
        if(manualHorizPos < 1.0) {
            state = State.MANUAL;
            manualHorizPos = lHoriz.getPosition() + 0.01;
        }
    }

    public void decrementHorizSlide() {
        if(manualHorizPos > 0.0) {
            state = State.MANUAL;
            manualHorizPos = lHoriz.getPosition() - 0.01;
        }
    }

    public void incrementRail() {
        if(manualRailPos < 1.0) {
            state = State.MANUAL;
            manualRailPos = rail.getPosition() + 0.01;
        }
    }

    public void decrementRail() {
        if(manualRailPos > 0.0) {
            state = State.MANUAL;
            manualRailPos = rail.getPosition() - 0.01;
        }
    }

    public void setPos(double pos) {
        state = State.MANUAL;
        manualHorizPos = 0.65 *(1-pos);
        manualRailPos = rail.getPosition();

    }

    public void setAutoPos(double pos) {
        state = State.MANUAL;
        if(pos > 100){
            manualHorizPos -= 0.0005;
        }
        else if (pos < -100){
            manualHorizPos += 0.0005;
        }

    }

    public void setTriggerPos(double trigger) {
        state = State.MANUAL;
        if(trigger < 0.15){
            manualHorizPos = 0.0;
            manualRailPos = trigger * 2 + 0.7;
        }
        else {
            manualHorizPos = trigger;
            manualRailPos = 1.0;
        }
    }


    private void setHorizPosition(double position) {
        lHoriz.setPosition(position);
        rHoriz.setPosition(position);
    }

    private void processState() {
        if (this.state != State.MANUAL) {
            setHorizPosition(this.state.slidePos);
            rail.setPosition(this.state.railPos);
        } else {
            lHoriz.setPosition(manualHorizPos);
            rHoriz.setPosition(manualHorizPos);
            rail.setPosition(manualRailPos);
        }
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();

        telemetry.addData("Horiz State: ", state);
    }
}
