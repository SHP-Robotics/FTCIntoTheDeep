package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kIncrement;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kLeftSlideName;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kMaxHeight;
import static org.firstinspires.ftc.teamcode.shplib.Constants.Vertical.kRightSlideName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shplib.Constants;
import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;

import dev.frozenmilk.dairy.cachinghardware.CachingCRServo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class VerticalSubsystem extends Subsystem {
    public final CachingDcMotorEx wormGear;
    private final CachingDcMotorEx linearSlide;
    private final CachingCRServo intake;
    private int slidePos;
    private int wormGearPos;
    private int offset;

    public enum State {
        BOTTOM(0, 0),
        DEPOSITING(1000, 1000),
        INTAKE(50, 50),
        LOWBUCKET(800, 800), // TODO: Tune
        HIGHBUCKET(3100, 3100), // TODO: Tune
        MANUAL(0, 0);

        final double slidePosition;
        final double wormGearPosition;

        State(double slidePosition, double wormGearPosition) {
            this.slidePosition = slidePosition;
            this.wormGearPosition = wormGearPosition;
        }
    }

    private State state;
    private State depositState;

    public VerticalSubsystem(HardwareMap hardwareMap) {
        slidePos = 0;
        wormGearPos = 0;
        offset = 0;

        wormGear = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("wormGear"));
        wormGear.setDirection(DcMotorSimple.Direction.FORWARD);
        wormGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        linearSlide = new CachingDcMotorEx((DcMotorEx) hardwareMap.get("linearSlide"));
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = new CachingCRServo(hardwareMap.crservo.get("intake"));
        intake.setPower(0);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        resetZeroPosition();

        setState(State.MANUAL);
        depositState = State.HIGHBUCKET;
    }

    public void setState(State slideState) {
        this.state = slideState;
    }

    public State getState() {
        return state;
    }

    public double getDriveBias() {
        return 0.6;
    }

    public double getSlidePosition() {
        return linearSlide.getCurrentPosition();
    }

    public double getWormGearPosition() {
        return wormGear.getCurrentPosition();
    }

    public void setSlidePosition(int position) {
        linearSlide.setTargetPosition(position);
//        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearSlide.setPower(Constants.Vertical.kRunPower);
    }

    public void setWormGearPosition(int position) {
        wormGear.setTargetPosition(position);
//        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wormGear.setPower(Constants.Vertical.kRunPower);
    }

    public void runIntake(double power) {
        intake.setPower(power);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void returnToIntake() {
        setState(State.INTAKE);
        setSlidePosition((int) State.INTAKE.slidePosition);
        setWormGearPosition((int) State.INTAKE.wormGearPosition);
    }

    public void incrementSlide() {
        if (slidePos <= kMaxHeight - kIncrement) {
            state = State.MANUAL;
            slidePos += kIncrement;
            setSlidePosition(slidePos);
        }
    }

    public void decrementSlide() {
        if (slidePos >= kIncrement) {
            state = State.MANUAL;
            slidePos -= kIncrement;
            setSlidePosition(slidePos);
        }
    }

//    public void incrementWormGear() {
//        wormGearPos += kIncrement;
//        setWormGearPosition(wormGearPos);
//    }
//
//    public void decrementWormGear() {
//        wormGearPos -= kIncrement;
//        setWormGearPosition(wormGearPos);
//    }

    public void cycleStates(boolean forward) {
        if (forward) {
            switch (depositState) {
                case HIGHBUCKET:
                    depositState = State.LOWBUCKET;
                    break;
                case LOWBUCKET:
                    depositState = State.HIGHBUCKET;
                    break;
            }
            setSlidePosition((int) depositState.slidePosition);
            setWormGearPosition((int) depositState.wormGearPosition);
        }
    }

    public void resetZeroPosition() {
        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        wormGear.setTargetPosition(0);
        linearSlide.setTargetPosition(0);

        linearSlide.setPower(Constants.Vertical.kRunPower);
//        wormGear.setPower(Constants.Vertical.kRunPower);

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidePos = 0;
        wormGearPos = 0;
        offset = 0;
    }

    @Override
    public void periodic(Telemetry telemetry) {
        telemetry.addData("State", state);
        telemetry.addData("Deposit State", depositState);
        telemetry.addData("Slide Position", getSlidePosition());
        telemetry.addData("Worm Gear Position", getWormGearPosition());
        telemetry.update();
    }
}