/* FTC Team 7572 - Version 1.2 (03/02/2022)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.lang.Math;

/**
 * This program implements robot movement based on Gyro heading and encoder counts.
 * It uses the Mecanumbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode and requires:
 * a) Drive motors with encoders
 * b) Encoder cables
 * c) Rev Robotics I2C IMU with name "imu"
 * d) Drive Motors have been configured such that a positive power command moves forward,
 *    and causes the encoders to count UP.
 * e) The robot must be stationary when the INIT button is pressed, to allow gyro calibration.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 */
@Autonomous(name="Autonomous Blue (ducks)", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousBducks extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y              = true;    // Drive forward/backward
    static final boolean DRIVE_X              = false;   // Drive right/left (not DRIVE_Y)

    static final double  DRIVE_SPEED_10       = 0.10;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_20       = 0.20;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_30       = 0.30;    // Lower speed for fine control going sideways
    static final double  DRIVE_SPEED_40       = 0.40;    // Normally go slower to achieve better accuracy
    static final double  DRIVE_SPEED_55       = 0.55;    // Somewhat longer distances, go a little faster
    static final double  TURN_SPEED_20        = 0.20;    // Nominal half speed for better accuracy.

    static final int     DRIVE_THRU           = 2;       // COAST after the specified movement

    double    sonarRangeL=0.0, sonarRangeR=0.0, sonarRangeF=0.0, sonarRangeB=0.0;

    OpenCvCamera webcam;
    public int hubLevel = 0;   // dynamic (gets updated every cycle during INIT)

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap,true);

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing webcam (please wait)");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.setPipeline(new FreightFrenzyPipeline(false, true));
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        int redAlignedCount;
        int blueAlignedCount;
		
        // Wait for the game to start (driver presses PLAY).  While waiting, poll for team color/number
        while (!isStarted()) {
            sonarRangeL = robot.updateSonarRangeL();
            telemetry.addData("ALLIANCE", "%s", "BLUE (ducks)");
            telemetry.addData("Hub Level", "%d", FreightFrenzyPipeline.hubLevel);
            telemetry.addData("Sonar Range", "%.1f inches (26.4)", sonarRangeL/2.54 );
            telemetry.addData("Left Blue Alignment", "%d %b", FreightFrenzyPipeline.leftBlueAverage, FreightFrenzyPipeline.alignedBlueLeft);
            telemetry.addData("Center Blue Alignment", "%d %b", FreightFrenzyPipeline.centerBlueAverage, FreightFrenzyPipeline.alignedBlueCenter);
            telemetry.addData("Right Blue Alignment", "%d %b", FreightFrenzyPipeline.rightBlueAverage, FreightFrenzyPipeline.alignedBlueRight);
            redAlignedCount = (FreightFrenzyPipeline.alignedRedLeft ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedCenter ? 1 : 0);
            redAlignedCount += (FreightFrenzyPipeline.alignedRedRight ? 1 : 0);
            blueAlignedCount = (FreightFrenzyPipeline.alignedBlueLeft ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueCenter ? 1 : 0);
            blueAlignedCount += (FreightFrenzyPipeline.alignedBlueRight ? 1 : 0);
            if(blueAlignedCount >= 2) {
                telemetry.addLine("Blue aligned for blue autonomous. Good job!");
                hubLevel = FreightFrenzyPipeline.hubLevel;
            } else if(redAlignedCount >= 2) {
                telemetry.addLine("****************************************************");
                telemetry.addLine("* WARNING: Red aligned for BLUE autonomous. *");
                telemetry.addLine("*          Something is wrong, so so wrong!             *");
                telemetry.addLine("****************************************************");
            } else {
                telemetry.addLine("Robot is not aligned for autonomous. Robot so confused!");
            }
            telemetry.update();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Sampling is completed during the INIT stage; No longer need camera active/streaming
        webcam.stopStreaming();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
            hubLevel = FreightFrenzyPipeline.hubLevel;
            FreightFrenzyPipeline.saveLastAutoImage();
        }

        webcam.closeCameraDevice();

        //---------------------------------------------------------------------------------
        // UNIT TEST: The following methods verify our basic robot actions.
        // Comment them out when not being tested.
//      testGyroDrive();
        //---------------------------------------------------------------------------------

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
        mainAutonomous();
        //---------------------------------------------------------------------------------

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    private void testGyroDrive() {
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, 24.0, 999.9, DRIVE_THRU ); // Drive FWD 24" along current heading
        gyroDrive(DRIVE_SPEED_30, DRIVE_X, 24.0, 999.9, DRIVE_THRU ); // Strafe RIGHT 24" along current heading
        gyroTurn(TURN_SPEED_20, (getAngle() + 90.0) );       // Turn CW 90 Degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Drive forward and collect the team element off the floor
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "collectTeamElement");
            telemetry.update();
            collectTeamElement( hubLevel );
        }

        // Drive to the alliance hub to deposit freight
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "moveToHub");
            telemetry.update();
            moveToHub( hubLevel );
        }

        // Deposit freight in top/middle/bottom
        if( opModeIsActive() ) {
            telemetry.addData("Skill", "dumpFreight");
            telemetry.update();
            dumpFreight( hubLevel );
        }

        // Drive to the duck carousel
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "spinDuckCarousel");
            telemetry.update();
            spinDuckCarousel( hubLevel );
        }

        // Drive to square to park
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "driveToSquare");
            telemetry.update();
            driveToSquare( hubLevel );
        }

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void collectTeamElement( int level ) {
        double turnAngle = 0.0;
        double distanceToGrab = 3.2;

        switch( level ) {
            case 3 : turnAngle = 0.0;      // right/top
                     distanceToGrab = -2.0;
                     break;
            case 2 : turnAngle = -25.0;    // middle/middle
                     distanceToGrab = -1.8;
                     break;
            case 1 : turnAngle = -42.0;
                     distanceToGrab = -4.0; // left/bottom
                     break;
        } // switch()

        // Move forward away from field wall so it's safe to raise the arms
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -4.2, 0.0, DRIVE_TO );

        // Command capping arm into the grabbing position
        robot.cappingArmPosInit( robot.CAPPING_ARM_POS_GRAB );
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_SPIN );

        // Process the first 750 msec of motion
        ElapsedTime fieldWallTimer = new ElapsedTime();
        fieldWallTimer.reset();  // start now
        while( opModeIsActive() && (fieldWallTimer.milliseconds() < 750) ) {
            performEveryLoop();
        }

        // We're now a safe distance from the wall to rotate the wrist and open the claw
        robot.clawServo.setPosition( robot.CLAW_SERVO_OPEN );    // open claw
        robot.wristPositionAuto( robot.WRIST_SERVO_GRAB );       // rotate wrist into the grab position
        robot.boxServo.setPosition( robot.BOX_SERVO_TRANSPORT );

        // Finish both arm movements before continuing
        while( opModeIsActive() &&
                ((robot.cappingMotorAuto == true) || (robot.freightMotorAuto == true)) ) {
            performEveryLoop();
        }

        // Turn toward the team element
        if( Math.abs(turnAngle) > 0.10 )
            gyroTurn(TURN_SPEED_20, turnAngle );

        // Drive forward to collect the element
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, distanceToGrab, 999.9, DRIVE_TO );
        robot.clawServo.setPosition( robot.CLAW_SERVO_CLOSED );    // close claw
        sleep( 500 );   // wait for claw to close

        // With the team element in hand, raise both arms (opposite directions)
        robot.cappingArmPosInit( robot.CAPPING_ARM_POS_LIBERTY );
        robot.wristPositionAuto( robot.WRIST_SERVO_LIBERTY );  // store position (handles unpowered!)
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_VERTICAL );

        // Finish both arm movements before continuing
        while( opModeIsActive() &&
                ((robot.cappingMotorAuto == true) || (robot.freightMotorAuto == true)) ) {
            performEveryLoop();
        }

    } // collectTeamElement

    /*--------------------------------------------------------------------------------------------*/
    private void moveToHub( int level ) {
        double angleToHub = 0.0;
        double distanceToHub = 0.0;
        double finalDistanceToHub = 0.0;
        int    freightArmPos = 0;

        switch( level ) {
            case 3 : angleToHub = -40.0;    // top
                     distanceToHub = -10.0;
                     finalDistanceToHub = 0.0;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_TOP_AUTO;
                     break;
            case 2 : angleToHub = -40.0;
                     distanceToHub = -6.0;  // middle
                     finalDistanceToHub = -3.0;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_MIDDLE_AUTO;
                     break;
            case 1 : angleToHub = -38.0;
                     distanceToHub = -6.0;  // bottom
                     finalDistanceToHub = -3.0;
                     freightArmPos = robot.FREIGHT_ARM_POS_HUB_BOTTOM_AUTO;
                     break;
        } // switch()

        // Start arm motion
        robot.freightArmPosInit( freightArmPos );

        // Turn toward hub
        double currentAngle = robot.headingIMU();
        if( Math.abs(angleToHub-currentAngle) > 2.0 )
            gyroTurn(TURN_SPEED_20, angleToHub );

        // Drive partially forward
        if( Math.abs(distanceToHub) > 0.0 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, distanceToHub, angleToHub, DRIVE_TO);
        }

        // Ensure arm has reached it's final position
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }

        // Drive forward the final amount
        if( Math.abs(finalDistanceToHub) > 0 ) {
            gyroDrive(DRIVE_SPEED_30, DRIVE_Y, finalDistanceToHub, angleToHub, DRIVE_TO );
        }

  } // moveToHub

    /*--------------------------------------------------------------------------------------------*/
    private void dumpFreight(int level ) {
        double servoPos = robot.BOX_SERVO_DUMP_TOP;
        double backDistance = 3.0;

        switch( level ) {
            case 3 : servoPos = robot.BOX_SERVO_DUMP_TOP;
                     backDistance = 5.2;
                     break;
            case 2 : servoPos = robot.BOX_SERVO_DUMP_MIDDLE;
                     backDistance = 5.0;
                     break;
            case 1 : servoPos = robot.BOX_SERVO_DUMP_BOTTOM;
                     backDistance = 6.0;
                     break;
        } // switch()

        robot.boxServo.setPosition( servoPos );     // rotate the box to dump
        sleep( 500 );                               // let cube drop out
        // back away and store arm
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, backDistance, 999.9, DRIVE_TO );
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_TRANSPORT1 );
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }
        robot.boxServo.setPosition( robot.BOX_SERVO_COLLECT );
    } // dumpFreight

    /*--------------------------------------------------------------------------------------------*/
    private void spinDuckCarousel( int level ) {
        double towardWall = 0;
        switch( level ) {
            case 3 : towardWall = 18.0; break; // top
            case 2 : towardWall = 19.0; break; // middle
            case 1 : towardWall = 16.5; break; // bottom
        } // switch()
        gyroTurn(TURN_SPEED_20, 90.0 );   // Turn toward wall
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -towardWall, 90.0, DRIVE_TO );
//      driveToBackDistance( 7.5, 1.0, 0.20, 10000 );
        double wallDistance = backRangeSensor()/2.54 - 7.5;
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, -wallDistance, 90.0, DRIVE_TO );
        gyroTurn(TURN_SPEED_20, 135.0 );   // Turn toward corner
        robot.duckMotor.setPower( -0.48 ); // Enable the carousel motor
        // We want to press against the carousel with out trying to reach a given point
        for( int loop=0; loop<5; loop++ ) {
            double barelyPressSpeed = 0.07;
            switch(loop) {
                case 0 : barelyPressSpeed = 0.07; break;
                case 1 : barelyPressSpeed = 0.06; break;
                case 2 : barelyPressSpeed = 0.03; break;
                case 3 : barelyPressSpeed = 0.02; break;
                case 4 : barelyPressSpeed = 0.02; break;
            }
            robot.driveTrainMotors( -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed, -barelyPressSpeed );
            sleep( 1000 );   // Spin the carousel for 5 seconds total
        } // loop
        robot.duckMotor.setPower( 0.0 );  // Disable carousel motor
    } // spinDuckCarousel

    /*--------------------------------------------------------------------------------------------*/
    private void driveToSquare( int level ) {
        gyroTurn(TURN_SPEED_20, 180.0 );   // Turn square to side wall
        double driveAwayFromWall = 29.0;
        switch( level ) {
            case 3 : driveAwayFromWall = 29.0; break; // top
            case 2 : driveAwayFromWall = 25.0; break; // middle
            case 1 : driveAwayFromWall = 25.2; break; // bottom
        } // switch()
        double squareDistance = driveAwayFromWall - backRangeSensor()/2.54;
        gyroDrive(DRIVE_SPEED_30, DRIVE_Y, squareDistance, 999.9, DRIVE_TO );
        // Don't lower arm to floor until we get into the square, in case the freight box has rotated
        // (the front edge will catch on the floor tile when we try to drive forward)
        robot.freightArmPosInit( robot.FREIGHT_ARM_POS_COLLECT );
        while( opModeIsActive() && (robot.freightMotorAuto == true) ) {
            performEveryLoop();
        }
        gyroDrive(DRIVE_SPEED_40, DRIVE_X, 5.0, 999.9, DRIVE_TO );
        // Until Autonomous ends (30 seconds), wait for arm to come down
        while( opModeIsActive() ) {
            sleep(75);  // wait for arm to lower
        }
    } // driveToSquare

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Capture range-sensor data (one reading! call from main control loop)  */
    /*                                                                                 */
    /*  Designed for test programs that are used to assess the mounting location of    */
    /*  your sensors and whether you get reliable/repeatable returns off various field */
    /*  elements.                                                                      */
    /*                                                                                 */
    /*  IMPORTANT!! updateSonarRangeL / updateSonarRangeR may call getDistanceSync(),  */
    /*  which sends out an ultrasonic pulse and SLEEPS for the sonar propogation delay */
    /*  (50 sec) before reading the range result.  Don't use in applications where an  */
    /*  extra 50/100 msec (ie, 1 or 2 sensors) in the loop time will create problems.  */
    /*  If getDistanceAsync() is used, then this warning doesn't apply.                */
    /*---------------------------------------------------------------------------------*/
    void processRangeSensors() {
        sonarRangeL = robot.updateSonarRangeL();
        sonarRangeR = robot.updateSonarRangeR();
        sonarRangeF = robot.updateSonarRangeF();
        sonarRangeB = robot.updateSonarRangeB();
    } // processRangeSensors

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: averaged range-sensor data (multiple readings!)                       */
    /*                                                                                 */
    /*  Designed for applications where continuous range updates are unnecessary, but  */
    /*  we want to know the correct distance "right now".                              */
    /*---------------------------------------------------------------------------------*/
    void averagedRangeSensors() {
        // repeatedly update all 4 readings.  Each loop adds a reading to the
        // internal array from which we return the new MEDIAN value.
        for( int i=0; i<5; i++ ) {
            sonarRangeL = robot.updateSonarRangeL();
            sonarRangeR = robot.updateSonarRangeR();
            sonarRangeF = robot.updateSonarRangeF();
            sonarRangeB = robot.updateSonarRangeB();
        }
    } // averagedRangeSensors

    /*---------------------------------------------------------------------------------*/
    double rightRangeSensor() {
        for( int i=0; i<5; i++ ) {
            sonarRangeR = robot.updateSonarRangeR();
            sleep(50);
        }
        return sonarRangeR;
    } // rightRangeSensor

    /*---------------------------------------------------------------------------------*/
    double backRangeSensor() {
        for( int i=0; i<6; i++ ) {
            sonarRangeB = robot.updateSonarRangeB();
            sleep(50);
        }
        return sonarRangeB;
    } // backRangeSensor

    /*---------------------------------------------------------------------------------*/
    double computeDriveAngle( double x0, double x1, double y0, double y1 ) {
        double deltaX = (x1 - x0);
        double deltaY = (y1 - y0);  // must drive at least 10 cm (also avoids trig error)
        double driveAngle = (deltaY < 10.0)? 0.0 : Math.atan2(deltaX,deltaY);  // radians
        driveAngle = driveAngle * (180.0 / Math.PI);  // degrees
        return driveAngle;
    } // computeDriveAngle

} /* AutonomousBducks */
