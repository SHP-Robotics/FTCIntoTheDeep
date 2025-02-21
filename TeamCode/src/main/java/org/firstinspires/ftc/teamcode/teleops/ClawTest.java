package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;

@Disabled
@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RotateSubsystem rotateSubsystem = new RotateSubsystem(hardwareMap);
        rotateSubsystem.setState(RotateSubsystem.State.INTAKE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            rotateSubsystem.rotatePos = -gamepad1.left_stick_y;
            telemetry.addData("rotate", rotateSubsystem.rotatePos);
            telemetry.update();
            rotateSubsystem.processState();
        }
    }
}