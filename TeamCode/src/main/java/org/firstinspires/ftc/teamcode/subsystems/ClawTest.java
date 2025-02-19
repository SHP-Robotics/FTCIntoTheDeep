package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ClawTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
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
