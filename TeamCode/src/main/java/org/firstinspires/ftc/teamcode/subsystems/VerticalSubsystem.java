package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kIncrement;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kRightSlideName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.algorithms.LowPassFilter;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class VerticalSubsystem extends Subsystem {
    private final CachingDcMotorEx leftSlide;
    private final CachingDcMotorEx rightSlide;
    private int slidePos;
    private int offset;
    private double slideVelocity;
    private LowPassFilter lowPassFilter;


    public enum State {
        BOTTOM(0),
        DEPOSITING(750),
        DOWN(100),
        DOWNAUTO(50),
        LOWBAR(0),
        HIGHBAR(1500),
        AUTOHIGHBAR(1050),
        LOWBUCKET(800),
        HIGHBUCKET(3250),
        MANUAL(0),
        NOPOWER(0);

        final double position;

        State(double position) {
            this.position = position;
        }
    }

    private State state, depositState;

    public VerticalSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;
        offset = 0;
        slideVelocity = 0;

        lowPassFilter = new LowPassFilter(0.5);

        leftSlide = new CachingDcMotorEx((DcMotorEx) hardwareMap.get(kLeftSlideName));
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightSlide = new CachingDcMotorEx((DcMotorEx) hardwareMap.get(kRightSlideName));
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetZeroPosition();

        setState(State.MANUAL);
        depositState = State.HIGHBAR;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public double getDriveBias(Boolean slow) {
        if(slow)
            return 0.4;
        return 0.7;
    }

    public double getSlidePosition() {
        return ((float)leftSlide.getCurrentPosition() + (float)rightSlide.getCurrentPosition()) / 2;
    }

    public void incrementSlide(){
        if(slidePos <= kMaxHeight - kIncrement) {
            state = State.MANUAL;
            slidePos += kIncrement;
        }
    }

    public void decrementSlide(){
        if(slidePos >= kIncrement ) {
            state = State.MANUAL;
            slidePos -= kIncrement;
        }
    }
    public void emergencyDecrementSlide(){
        state = State.MANUAL;
        slidePos -= kIncrement;
    }
    public void endReset(){
        offset = (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;
        slidePos = 0;
    }
    public void resetZeroPosition() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        rightSlide.setPower(Constants.Vertical.kRunPower);
        leftSlide.setPower(Constants.Vertical.kRunPower);

        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateSlidePower(){
        if(state == State.BOTTOM
                && (depositState == State.HIGHBUCKET || depositState == State.LOWBUCKET)
                && slideVelocity > 250.0
                && rightSlide.getCurrentPosition() > 100) {
                rightSlide.setPower(0);
                leftSlide.setPower(0);
        }
        else {
            rightSlide.setPower(Constants.Vertical.kRunPower);
            leftSlide.setPower(Constants.Vertical.kRunPower);
        }
    }
    public void updateSlideVelocity(){
        double currentPos = (Math.abs(rightSlide.getVelocity()) + Math.abs(leftSlide.getVelocity()))/2;
        slideVelocity = lowPassFilter.forward(currentPos);
    }

    public void setDepositState(State state){
        this.depositState = state;
    }
    private void setPosition(double position){
        rightSlide.setTargetPosition((int) position);
        leftSlide.setTargetPosition((int) position);
    }

    private void processState() {
        updateSlideVelocity();
        updateSlidePower();
        if (this.state == State.MANUAL) {
            this.setPosition(slidePos+offset);
            return;
        }
        else if (this.state == State.DEPOSITING){
            this.setPosition(this.depositState.position+offset);
            return;
        }
        else if (this.state == State.BOTTOM){
            this.setPosition(this.state.position);
            return;
        }
        else {
            this.setPosition(this.state.position + offset);
        }

    }

    @Override
    public void periodic(Telemetry telemetry) {
        processState();
        telemetry.addData("DEPOSIT STATE:", depositState);
        telemetry.addData("Slide Velocity:", rightSlide.getVelocity());
        telemetry.addData("Slide State: ", state);
        telemetry.addData("Left Slide Position: ", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position: ", rightSlide.getCurrentPosition());
    }
}
