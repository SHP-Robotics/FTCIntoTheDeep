package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class LConstants {
    static {
        // START -- AUTO FROM PESTO
        ThreeWheelConstants.forwardTicksToInches = 1 / PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;
        ThreeWheelConstants.strafeTicksToInches = 1 / PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;
        ThreeWheelConstants.turnTicksToInches = 1 / PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;

        // NOT COMPATIBLE?
        ThreeWheelConstants.leftY = 7.17891; // PestoFTCConfig.ODOMETRY_WIDTH / 2;
        ThreeWheelConstants.rightY = -7.17891; // -PestoFTCConfig.ODOMETRY_WIDTH / 2;

        ThreeWheelConstants.strafeX = 0 ; // PestoFTCConfig.FORWARD_OFFSET;

        ThreeWheelConstants.leftEncoder_HardwareMapName = PestoFTCConfig.leftName;
        ThreeWheelConstants.rightEncoder_HardwareMapName = PestoFTCConfig.rightName;
        ThreeWheelConstants.strafeEncoder_HardwareMapName = PestoFTCConfig.centerName;
        // END -- AUTO FROM PESTO

        // REVERSE OF PESTO
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;

        // SAME AS PESTO
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




