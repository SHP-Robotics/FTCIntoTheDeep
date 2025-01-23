package org.firstinspires.ftc.teamcode;


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
                clawSubsystem.update();
                wormGearSubsystem.setToZero(touchSensor, telemetry);
            }

            if (gamepad1.b) {
                teleOpController.resetIMU();
                tracker.reset();
            }

            if (gamepadInterface.isKeyUp(GamepadKey.LEFT_BUMPER)) {
                wormGearSubsystem.cycle();
                viperSlideSubsystem.cycle();

            }
            if (gamepadInterface.isKeyDown(GamepadKey.RIGHT_BUMPER)) {
                wormGearSubsystem.cycleHanging();
                viperSlideSubsystem.cycleHanging();

            }
            if (gamepad1.dpad_right) {
                WormGearSubsystem.intakeUp = false;
                elapsedTime.reset();
                wormGearSubsystem.update();
                if (wormGearSubsystem.mode== WormGearSubsystem.WormMode.INTAKE){


                while (elapsedTime.milliseconds()<400){

                    }
                }
                clawSubsystem.setClose();

            }
            if (gamepad1.dpad_left) {
                WormGearSubsystem.intakeUp = true;
                wormGearSubsystem.update();
                clawSubsystem.setOpen();

            }
            if (gamepad1.left_trigger>0.9) {
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
            wristSubsystem.update();
            if (wormGearSubsystem.zeroed) {
                if (wormGearSubsystem.hangMode == WormGearSubsystem.HangMode.NONE) {
                    wormGearSubsystem.update();
                    viperSlideSubsystem.update();
                    if (ViperSlideSubsystem.mode == ViperSlideSubsystem.ViperMode.DRIVING2){
                        wristSubsystem.reset();
                    }
                } else {
                    wormGearSubsystem.updateHanging();
                    viperSlideSubsystem.updateHanging();


                }
            }
        }

    }
}


