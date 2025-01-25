package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class WormGear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor wormgear = (DcMotor) hardwareMap.get("wormGear");
        wormgear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wormgear.setDirection(DcMotorSimple.Direction.FORWARD);
        wormgear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            wormgear.setPower(-gamepad1.left_stick_y);
            telemetry.addData("power", -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
