package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kNeutral;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kPickup;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Rotate.kRotateName;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class RotateSubsystem extends Subsystem {
    private final CachingServo rotate;
    public double rotatePos;
    public boolean aligned;

    public enum State {
        DROPOFF,
        INTAKE,
        PICKUP,
        SAMPLE,
        DROPOFF_BUCKET,
        ANTI_SAMPLE,
        NEUTRAL;
    }
    private State state;

    public RotateSubsystem(HardwareMap hardwareMap){
        rotate = new CachingServo((Servo) hardwareMap.get(kRotateName));
        rotate.setDirection(Servo.Direction.REVERSE);
        rotatePos = kNeutral;
        aligned = false;
        setState(State.NEUTRAL);
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setPos(double pos) {
        rotate.setPosition(pos);
    }

    public void rotateCW() {
        //if(state == State.INTAKE){
        state = State.INTAKE;
        rotatePos += 0.2;
        //}
    }
    public void rotateCCW() {
        //if (state == State.INTAKE) {
        state = State.INTAKE;
        rotatePos -= 0.2;
       // }
    }

    public void turn(double radians){
        this.state = State.INTAKE;
        double prevPos = rotatePos;

//        rotatePos = rotatePos*0.5 + ((radians*0.6/PI)+0.4)*0.5;
        rotatePos = (radians*0.656/PI)+0.398;

        aligned = Math.abs(prevPos - rotatePos) < 0.1;

    }

    public void processState() {
        if (this.state == State.NEUTRAL || this.state == State.DROPOFF)
            rotatePos = kNeutral;
        else if (this.state == State.PICKUP)
            rotatePos = kPickup;
        else if (this.state == State.DROPOFF_BUCKET)
            rotatePos = 0.425; //0.625;
        else if (this.state == State.ANTI_SAMPLE)
            rotatePos = 0.55;
        else if (this.state == State.SAMPLE)
            rotatePos = 0.9; // 1;

        rotate.setPosition(rotatePos);
    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();

        telemetry.addData("Rotate State: ", state);
    }
}
