package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.FINISH;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.NONE;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.SETUP;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.VIPERDOWN;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.VIPERUP;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.WORMGEARBACK;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.WORMGEARFOWARD;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.DRIVING;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.DRIVING2;

import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.INTAKE;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.WormMode.OUTTAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class WormGearSubsystem {
    private static final int OFFSET =-1550;
    public static boolean intakeUp = true;
    public static boolean downExtra = false;

    boolean  zeroed=false;
    public enum WormMode {
        DRIVING (OFFSET),

        INTAKE (OFFSET-2270),
        DRIVING2 (OFFSET),

        OUTTAKE (OFFSET-120);



        WormMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }
    public enum HangMode {

        NONE (OFFSET),
        SETUP (OFFSET-400),
        VIPERDOWN (OFFSET-400),
        WORMGEARBACK (-150),
        WORMGEARFOWARD (OFFSET),
        VIPERUP (OFFSET),
        FINISH (OFFSET);
        HangMode(int position) {
            this.position = position;
        }

        public int getPosition() {
            return this.position;
        }

        private final int position;
    }

    private DcMotorEx wormGear;
    public WormMode mode= DRIVING2;
    public HangMode hangMode = NONE;

    public WormGearSubsystem(HardwareMap hardwareMap) {
        wormGear = hardwareMap.get(DcMotorEx.class, "WormGear");
        wormGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wormGear.setTargetPosition(wormGear.getCurrentPosition());
    }

    public void cycle() {
        switch (mode) {
            case DRIVING:
                mode = INTAKE;
                break;

            case INTAKE:
                mode = DRIVING2;
                break;

            case DRIVING2:
                mode = OUTTAKE;
                break;
            case OUTTAKE:
                mode = DRIVING;
                break;

        }
        update();

    }
    public void cycleHanging() {
        switch (hangMode) {
            case NONE:
                hangMode = SETUP;
                break;
            case SETUP:
                hangMode = VIPERDOWN;
                break;
            case VIPERDOWN:
                hangMode = WORMGEARBACK;
                break;
            case WORMGEARBACK:
                hangMode = WORMGEARFOWARD;
                break;
            case WORMGEARFOWARD:
                hangMode = VIPERUP;
                break;
            case VIPERUP:
                hangMode = FINISH;
                break;
            case FINISH:
                hangMode = NONE;
                break;
        }
        update();
    }

    public void reset() {
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setToZero(TouchSensor touchSensor, Telemetry telemetry) {
        wormGear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wormGear.setPower(1);
        while(!touchSensor.isPressed()){
            updateTelemetry(telemetry);
            telemetry.update();
        }
        reset();
        updateTelemetry(telemetry);
        telemetry.update();
        wormGear.setPower(0);
        wormGear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wormGear.setTargetPosition(OFFSET);
        wormGear.setPower(1);
        while(wormGear.getCurrentPosition()>OFFSET+10){
        }
        zeroed=true;


    }
    public void update() {
        if( hangMode== NONE && zeroed){

            if (intakeUp && mode==INTAKE) {
                if (downExtra) {
                    wormGear.setTargetPosition(mode.getPosition() + 50);

                }else {
                    wormGear.setTargetPosition(mode.getPosition() + 150);
                }
            }else{
                if (downExtra) {
                    wormGear.setTargetPosition(mode.getPosition() - 100);

                }else {
                    wormGear.setTargetPosition(mode.getPosition() - 50);
                }
            }
            wormGear.setPower(1);

        }
    }
    public void updateHanging() {

            wormGear.setTargetPosition(hangMode.getPosition());
            wormGear.setPower(0.6);

    }

    public void resetCycles(){
        hangMode=NONE;
        mode=WormMode.DRIVING;
    }
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("WormGear Mode", mode);
        telemetry.addData("WORM HANG Mode", hangMode);

        telemetry.addData("WormGear Position", wormGear.getCurrentPosition());
    }

}
