package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClawName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kClose;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kLedName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Claw.kOpen;
import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.State.OPEN;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class ClawSubsystem extends Subsystem {
    private final CachingServo claw;
    private final CachingServo ledLight;
    private final DigitalChannel breakBeam;
    boolean blockInClaw;
    boolean disableBreakBeam;

    public enum ColorState {
        OFF(0.0),
        RED(0.279),
        PINK(0.7),
        GREEN(0.5);
        final double color;

        ColorState(double color) {
            this.color = color;
        }
    }

    private ColorState colorState;

    public enum State {
        OPEN,
        CLOSE,
        MANUAL;
    }

    private State state;

    public ClawSubsystem(HardwareMap hardwareMap) {
        claw = new CachingServo((Servo) hardwareMap.get(kClawName));
        ledLight = new CachingServo((Servo) hardwareMap.get(kLedName));
        setColor(ColorState.OFF);

        breakBeam = hardwareMap.digitalChannel.get("breakBeam");
        blockInClaw = false;
        setState(State.CLOSE);

        disableBreakBeam = false;
    }

    public boolean isBlockInClaw() {
        return blockInClaw;
    }

    public void setColor(ColorState color) {
        colorState = color;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void increment() {
        state = State.MANUAL;
        claw.setPosition(claw.getPosition() + 0.01);
    }

    public void decrement() {
        state = State.MANUAL;
        claw.setPosition(claw.getPosition() - 0.01);
    }

    public void open() {
        state = State.OPEN;
        claw.setPosition(kOpen);
    }

    public void close() {
        state = State.CLOSE;
        claw.setPosition(kClose);
    }

    public void toggleBreakBeam(){
        disableBreakBeam = !disableBreakBeam;
    }

    private void processState() {
        if (this.state == State.CLOSE)
            claw.setPosition(kClose);
        else if (this.state == OPEN)
            claw.setPosition(kOpen);

        updateBreakBeam();

        if(colorState == ColorState.OFF)
            ledLight.setPosition(ColorState.OFF.color);
        else if(colorState == ColorState.RED)
            ledLight.setPosition(colorState.color);
        else if(colorState == ColorState.PINK)
            ledLight.setPosition(ColorState.PINK.color);
        else if(colorState == ColorState.GREEN)
            ledLight.setPosition(ColorState.GREEN.color);
    }
    private void updateBreakBeam(){
        //true is block in claw, false is no block
        blockInClaw = (state == State.CLOSE && !breakBeam.getState()) || disableBreakBeam;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();

        telemetry.addData("Break Beam: ", breakBeam.getState());
        telemetry.addData("Claw State: ", state);
    }
}
