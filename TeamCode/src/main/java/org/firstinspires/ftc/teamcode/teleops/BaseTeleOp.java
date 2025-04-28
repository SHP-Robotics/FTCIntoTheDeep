package org.firstinspires.ftc.teamcode.teleops;

import com.shprobotics.pestocore.devices.GamepadInterface;

import org.firstinspires.ftc.teamcode.shplib.BaseRobot;
import org.firstinspires.ftc.teamcode.shplib.commands.RunCommand;

public class BaseTeleOp extends BaseRobot {
    private double driveBias;
    GamepadInterface gamepadInterface1, gamepadInterface2;
    @Override
    public void init(){
        super.init();
        drive.setDefaultCommand(
                new RunCommand(
                        () -> drive.mecanum(-driveBias*gamepad1.left_stick_y, driveBias*gamepad1.left_stick_x, driveBias*gamepad1.right_stick_x)
                )
        );

        gamepadInterface1 = new GamepadInterface(gamepad1);
        gamepadInterface2 = new GamepadInterface(gamepad2);
    }

    @Override
    public void start(){
        super.start();
//        driveBias = vertical.getDriveBias(gamepad1.right_stick_button);
        driveBias = 1;
    }

    @Override
    public void loop(){
        super.loop();
//        driveBias = vertical.getDriveBias(gamepad1.right_stick_button);
        gamepadInterface1.update();
        drive.update(gamepad1);

        gamepadInterface2.update();
        drive.update(gamepad2);

//        new Trigger(gamepadInterface1.isKeyDown(GamepadKey.RIGHT_BUMPER) && pivot.getState() != PREPARE_INTAKE,
//                new DriveToSubCommand(rotate, claw, pivot, horiz)
//                        .then(new RunCommand(()->{
//                            clawAlignment.reset();
//                            rotationBias = 0.5;
//                        })));
//
    }


}