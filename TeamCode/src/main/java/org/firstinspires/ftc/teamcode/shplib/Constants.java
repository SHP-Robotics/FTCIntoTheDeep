package org.firstinspires.ftc.teamcode.shplib;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    // Target voltage for voltage compensation
    public static final double kNominalVoltage = 13.0;


    public static final class Sensors{
        public static final String kClawColorName = "clawColor";
    }
    public static final class Pivot{

        public static final String kWristName = "wrist";
        public static final String klElbowName = "lElbow";
        public static final String krElbowName = "rElbow";
    }

    public static final class Vertical {
        public static final String kLeftSlideName = "leftVSlide";
        public static final String kRightSlideName = "rightVSlide";

        public static final double kMaxHeight = 3200;
        public static final double kIncrement = 25;
        public static final double kRunPower = 0.75;
    }
}
