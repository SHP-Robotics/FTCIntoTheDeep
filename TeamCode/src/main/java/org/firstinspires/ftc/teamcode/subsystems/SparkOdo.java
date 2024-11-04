package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

@Config
public class SparkOdo implements LocalizerInterface {

    //X+ Axis --> Forward
    //Y+ Axis --> Left

    //Position offset of the Spark:
    public static double X_OFFSET = -2; //Inches
    public static double Y_OFFSET = -0.6; //Inches

    public static double HEADING_OFFSET = 180; //Degrees
    public double weight = 0.33;
    private SparkFunOTOS odo;

    private SparkFunOTOS.Pose2D lastPos, totalDist;

    /**
     * Quick Constructor for Spark Odo Class
     * @param hw [HardwareMap] Hardware map required to initialize sensors.
     */
    public SparkOdo(HardwareMap hw){
        this(hw, "spark");
    }

    /**
     * Constructor for Spark Odo Class
     * @param hw [HardwareMap] Hardware map required to initialize sensors
     * @param name [String] Name used in configuration for spark odo
     */
    public SparkOdo(HardwareMap hw, String name){
        odo = hw.get(SparkFunOTOS.class, name);
        configureOtos();
        lastPos = odo.getPosition();
        totalDist = new SparkFunOTOS.Pose2D();
    }

    /**
     * Reset the tracking if the user requests it
     */
    public void resetOdo(){
        odo.resetTracking();
    }

    /**
     * Re-calibrate the IMU if the user requests it
     */
    public void calibrateOdo(){
        odo.calibrateImu();
    }

    /**
     * @return [Pose2D] Returns data related to position and orientation
     */
    public SparkFunOTOS.Pose2D getPos(){
        return odo.getPosition();
    }

    /**
     * Will be used to track total distance throught to be traveled vs actual distance traveled
     * This is primarily a tester function, will be made private/deleted in the future
     * @return [Pose2D] Returns total distance/rotation traveled
     */
    public SparkFunOTOS.Pose2D updateTotalDist(){
        SparkFunOTOS.Pose2D currentPos = getPos();
        //Get Delta X
        totalDist.x += Math.abs(currentPos.x - lastPos.x);
        //Get Delta Y
        totalDist.y += Math.abs(currentPos.y - lastPos.y);
        //Get Delta H
        totalDist.h += Math.abs(currentPos.h - lastPos.h);
        lastPos = currentPos;
        return totalDist;
    }


    /**
     * Helper function to set up the Spark Odometry
     */
    private void configureOtos() {
//        telemetry.addLine("Configuring OTOS...");
//        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        odo.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        odo.setAngularUnit(AngleUnit.RADIANS);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(X_OFFSET, Y_OFFSET, Math.toRadians(HEADING_OFFSET));
        odo.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        odo.setLinearScalar(1.0);
        odo.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        odo.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        odo.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odo.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        odo.getVersionInfo(hwVersion, fwVersion);

//        telemetry.addLine("OTOS configured! Press start to get position data!");
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        telemetry.update();
    }

    /**
     * @return [double] Returns a weight associated with the localizer
     */
    @Override
    public double getWeight() {
        return weight;
    }

    /**
     * Not to be confused with getPos(). getPos() returns a Pose2D, this returns a Pose2d.
     * @return [Pose2d] Returns position and orientation data
     */
    @Override
    public Pose2d getPosition() {
        SparkFunOTOS.Pose2D result = getPos();
        return new Pose2d(-result.x, result.y, -result.h);
    }

    public String toString(){
        Pose2d pos = getPosition();
        return String.format("X: %f\nY: %f\nHeading: %f",
                pos.position.x,
                pos.position.y,
                Math.toDegrees(pos.heading.toDouble()));
    }
}