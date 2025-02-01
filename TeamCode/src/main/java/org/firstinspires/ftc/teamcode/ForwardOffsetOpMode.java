package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.tuners.ForwardOffsetTuner;

@TeleOp
public class ForwardOffsetOpMode extends ForwardOffsetTuner {
    @Override
    public void setTracker(HardwareMap hardwareMap) {
        tracker = (ThreeWheelOdometryTracker) PestoFTCConfig.getTracker(hardwareMap);
    }
}
