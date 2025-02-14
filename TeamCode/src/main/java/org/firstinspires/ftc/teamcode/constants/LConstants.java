package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;

public class LConstants {
    static {
        // START -- AUTO FROM PESTO
        ThreeWheelConstants.forwardTicksToInches = 1 / PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;
        ThreeWheelConstants.strafeTicksToInches = 1 / PestoFTCConfig.ODOMETRY_TICKS_PER_INCH;
        ThreeWheelConstants.turnTicksToInches = 0.0026; // TODO: USE TURN AUTONOMOUS TUNER, DO CALCULATIONS BY SELF

        // NOT COMPATIBLE?
        ThreeWheelConstants.leftY = PestoFTCConfig.ODOMETRY_WIDTH / 2;
        ThreeWheelConstants.rightY = -PestoFTCConfig.ODOMETRY_WIDTH / 2;

        // TODO: CHECK THIS
        ThreeWheelConstants.strafeX = -PestoFTCConfig.FORWARD_OFFSET;

        ThreeWheelConstants.leftEncoder_HardwareMapName = PestoFTCConfig.leftName;
        ThreeWheelConstants.rightEncoder_HardwareMapName = PestoFTCConfig.rightName;
        ThreeWheelConstants.strafeEncoder_HardwareMapName = PestoFTCConfig.centerName;
        // END -- AUTO FROM PESTO

        // SAME AS PESTO
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;

        // REVERSE OF PESTO
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




