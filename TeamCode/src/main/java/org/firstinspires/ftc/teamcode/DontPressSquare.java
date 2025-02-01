package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.*;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.HangMode;
import static org.firstinspires.ftc.teamcode.ViperSlideSubsystem.ViperMode.*;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.*;
import static org.firstinspires.ftc.teamcode.WormGearSubsystem.HangMode.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;


@TeleOp
public class DontPressSquare extends LinearOpMode {
    MecanumController mecanumController;
    DeterministicTracker tracker;
    TeleOpController teleOpController;
    ViperSlideSubsystem viperSlideSubsystem;
    ClawSubsystem clawSubsystem;
    WristSubsystem wristSubsystem;

    WormGearSubsystem wormGearSubsystem;
    GamepadInterface gamepadInterface;
    GamepadInterface gamepad2Interface;

    TouchSensor touchSensor;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        teleOpController = PestoFTCConfig.getTeleOpController(mecanumController, tracker, hardwareMap);
        viperSlideSubsystem = new ViperSlideSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);
        wristSubsystem = new WristSubsystem(hardwareMap);
        wormGearSubsystem = new WormGearSubsystem(hardwareMap);
        wormGearSubsystem.reset();
        elapsedTime=new ElapsedTime();
        gamepadInterface = new GamepadInterface(gamepad1);
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        boolean IMU=true;
        waitForStart();

        teleOpController.resetIMU();
        tracker.reset();

        while (opModeIsActive()) {
            wormGearSubsystem.updateTelemetry(telemetry);
            viperSlideSubsystem.updateTelemetry(telemetry);
            clawSubsystem.updateTelemetry(telemetry);
            wristSubsystem.updateTelemetry(telemetry);

            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("HeadingRadians", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
            tracker.update();
            gamepadInterface.update();
            teleOpController.updateSpeed(gamepad1);

            if (gamepad1.right_trigger > 0.9) {
                teleOpController.driveFieldCentric(-gamepad1.left_stick_y * 0.35, gamepad1.left_stick_x * 0.35, gamepad1.right_stick_x * 0.35);
            } else {
                teleOpController.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            if (gamepad1.x) {
                wormGearSubsystem.mode= WormMode.DRIVING2;
                ViperSlideSubsystem.mode= ViperSlideSubsystem.ViperMode.DRIVING2;
                wormGearSubsystem.hangMode= NONE;
                hangMode= HangMode.NONE;
                wormGearSubsystem.setToZero(touchSensor, telemetry);
                clawSubsystem.setClose();
                clawSubsystem.update();
            }

            if (gamepad1.b) {
                teleOpController.resetIMU();
                tracker.reset();
            }

            if (gamepadInterface.isKeyUp(GamepadKey.LEFT_BUMPER)) {
                wormGearSubsystem.cycle();
                viperSlideSubsystem.cycle();
                if (wormGearSubsystem.mode == WormMode.INTAKE) {
                    intakeUp = true;
                    intakeSlideExtend = false;

                }
            }
            if (gamepadInterface.isKeyDown(GamepadKey.RIGHT_BUMPER)) {
                    viperSlideSubsystem.cycleHanging();
                    wormGearSubsystem.cycleHanging();
                    wormGearSubsystem.updateHanging();
                    viperSlideSubsystem.updateHanging();
                    if (wormGearSubsystem.hangMode==VIPERDOWN){
                        viperSlideSubsystem.updateHanging();
                        viperSlideSubsystem.resetCycles();
                        wormGearSubsystem.resetCycles();
                        elapsedTime.reset();
                        while (elapsedTime.seconds()<0.3){
                        }
                        clawSubsystem.setOpen();
                    }
                    if (wormGearSubsystem.hangMode==SETUP){
                        wristSubsystem.reset(0.8);
                        clawSubsystem.setClose();
                    }
            }
            if (gamepadInterface.isKeyUp(GamepadKey.Y)) {
                wormGearSubsystem.cycleHanging();
                viperSlideSubsystem.cycleHanging();
            }

            if (gamepad2.x) {

                viperSlideSubsystem.downAndZero();
            }

            if (gamepad1.dpad_left) {
                while(gamepad1.dpad_left){
                }
                if(clawSubsystem.mode==ClawSubsystem.ClawMode.OPEN) {
                    intakeUp = false;
                    elapsedTime.reset();
                    downExtra = intakeSlideExtend;
                    wormGearSubsystem.update();
                    if (wormGearSubsystem.mode == WormMode.INTAKE) {
                        while (elapsedTime.milliseconds() < 300) {
                        }
                    }
                    clawSubsystem.setClose();
                }else {
                    intakeUp = true;
                    wormGearSubsystem.update();
                    clawSubsystem.setOpen();
                }
            }
            if (gamepad1.dpad_right) {
                while(gamepad1.dpad_right){
                }
                intakeSlideExtend = !intakeSlideExtend;
                viperSlideSubsystem.update();
                viperSlideSubsystem.switchPower();
            }
            if (gamepad1.left_trigger>0.8) {
                gamepad1.rumble(500);
                wormGearSubsystem.resetCycles();
                viperSlideSubsystem.resetCycles();
            }
            if (gamepad1.dpad_down) {
                while (gamepad1.dpad_down){}
                wristSubsystem.incrementAdd();
            }
            if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up){}
                wristSubsystem.incrementSubtract();
            }
            if (gamepad1.a) {
                viperSlideSubsystem.zero();

            }

                if (gamepad2.a){
                while(gamepad2.a){

                }

                    if(!IMU) {
                        teleOpController.useIMU();
                        teleOpController.resetIMU();
                        IMU = true;
                    }
                    else {
                        teleOpController.useTrackerIMU(tracker);
                        IMU = false;
                    }
            }
            wristSubsystem.update();
            if (wormGearSubsystem.zeroed) {
                if (wormGearSubsystem.hangMode == NONE) {

                    wormGearSubsystem.update();
                    viperSlideSubsystem.update();
                    if (mode == DRIVING2){
                        intakeUp = true;
                        wristSubsystem.reset(0.45);
                    }
                    if (mode == DRIVING){
                        wristSubsystem.reset(0.8);
                    }
                } else {
                    wormGearSubsystem.updateHanging();
                    viperSlideSubsystem.updateHanging();
                }
            }
        }
    }

}


