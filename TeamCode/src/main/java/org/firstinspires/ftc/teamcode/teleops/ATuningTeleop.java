package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;
import org.firstinspires.ftc.teamcode.shplib.commands.Trigger;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

@TeleOp
public class ATuningTeleop extends BaseRobot {

    private double debounce;
    GamepadInterface gamepadInterface2;

    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)
                )
        );

        gamepadInterface2 = new GamepadInterface(gamepad2);

    }
    @Override
    public void start(){
        super.start();
        debounce = Clock.now();
    }

    @Override
    public void loop(){
        super.loop();
        drive.update(gamepad2);


        new Trigger(gamepad1.dpad_up, new RunCommand(()-> {
            pivot.incrementElbowUp();
        }));
        new Trigger(gamepad1.dpad_down, new RunCommand(()-> {
            pivot.decrementElbowDown();
        }));

        new Trigger(gamepad1.dpad_right, new RunCommand(() ->{
            pivot.incrementWristUp();
        }));
        new Trigger(gamepad1.dpad_left, new RunCommand(() ->{
            pivot.decrementWristDown();
        }));


        new Trigger(gamepad1.right_trigger > 0.1, new RunCommand(() -> {
            vertical.incrementSlide();
        }));
        new Trigger(gamepad1.left_trigger > 0.1, new RunCommand(() -> {
            vertical.decrementSlide();
        }));

        new Trigger(gamepad1.circle, new RunCommand(() -> {
            horiz.incrementHorizSlide();
        }));
        new Trigger(gamepad1.square, new RunCommand(() -> {
            horiz.decrementHorizSlide();
        }));
        new Trigger(gamepad1.triangle, new RunCommand(() -> {
            horiz.incrementRail();
        }));
        new Trigger(gamepad1.cross, new RunCommand(() -> {
            horiz.decrementRail();
        }));

        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_LEFT), new RunCommand(() -> {
            rotate.rotateCCW();
        }));
        new Trigger(gamepadInterface2.isKeyDown(GamepadKey.DPAD_RIGHT), new RunCommand(() -> {
            rotate.rotateCW();
        }));

        new Trigger(gamepad2.circle, new RunCommand(() -> {
            claw.increment();
        }));new Trigger(gamepad2.square, new RunCommand(() -> {
            claw.decrement();
        }));



        debounce = Clock.now();

    }



}
