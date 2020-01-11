package org.firstinspires.ftc.teamcode;


//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class Maincanum extends LinearOpMode {



    // Declare motors here
    DcMotor leftFront; //Drivetrain motor
    DcMotor leftBack; //Drivetrain motor
    DcMotor rightFront; //Drivetrain motor
    DcMotor rightBack; //Drivetrain motor

    DcMotor liftExtender; //forklift extender motor
    DcMotor liftRaise;

    //declare sensor
    BNO055IMU IMU;

    //declare servo
    Servo rightFGrabber;
    Servo leftFGrabber;

    Servo blockgrabFore;
    Servo blockgrabAft;

    /*vision doge, bad

    public OpenCvCamera phoneCam;
    public SkystoneDetector skyStoneDetector;

*/


    //Declare variables here:



    int counts; //Variable to represent encoder counts

    double liftPos = 0; //variable for raising main forklift

    static final double COUNTS_PER_MOTOR_REV = 537.6; //NeveRest 20: 537.6, NeveRest 40: 1120, NeveRest 60: 1680, Bare NeveRest: 103
    static final double DRIVE_GEAR_REDUCTION = 1.0; //2.0 would be for speed, .5 would be for torque
    static final double WHEEL_DIAMETER_INCHES = 4.0; //If you manage to find wheels other than 4" change this
    static final double DRIVE_NORMAL_ACCEL = 2; // if the robot is overshooting regular drive, increase this
    static final double DRIVE_STRAFE_ACCEL = 2; // if the robot is overshooting strafe drive, increase this

    //below 4 part of mecTeleOp
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SCALEDPOWER = 1; //Emphasis on current controller reading (vs current motor power) on the drive train
    public static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    int driveBase = 1; //Set to 0 for six-wheel-drive, or 1 for mecanum

    double turningThreshold = 1.5; //# of degrees robot is allowed to be off by, increase to allow more room for error, decrease to be more precise
    double drivingThreshold = 10; //# of counts robot is allowed to be off by, increase to allow more room for error, decrease to be more precise
    double strafingThreshold = 10; // same as above but specified for strafing for extra precision

    double turningPower = 0.4; //Tune up if the robot needs to turn faster, and lower to turn slower

    double drivingPower = 1; //Tune up if the robot needs to drive faster, and lower to drive slower
    double strafingPower = 1; //tune up if strafing is too slow, in ~.25 increments

    boolean strafeRight = true;

    int detectedSkystone; //auto block target.


    //output from mecanum drive code, before trigger slowdown
    double leftFrontMecanum = 0;
    double rightFrontMecanum = 0;
    double leftBackMecanum = 0;
    double rightBackMecanum = 0;



    int blockLocation;

    //--------------- VuForia ---------------

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    public static final String VUFORIA_KEY = "AZuy3uj/////AAABmZFNolBNLkzco28WPYoY9aB7osTlNrYsmY5HNInnQWmarXmArRFAbwxKFzCh30cG02LXbQyLgaHjQlFtneLVgfoaIeGCK7qTPrNWt/APfScb2lNP0lrLHDjZvheASKuOWCXpKe/qtHFUEMHq7FFUCVnmEV5lP86+NKL5Lj5fbqUJjxHobiYmJ2BnKJzy/9wOLfeWN29LEkFa337czUcuNGgT1pdBG3tfjotVH0kx4e5jZyj5gf+7893QPBYJgTuaO6IPZkxtLCR+Xn8Y8vhL17944ZxgqOh2TMk55OdsSw8EOTUvNf5e1jIyMNMPcG1o2BOY6/GUq/wykNwQtqwVXhxYoeZuTEAEUxvtOHIAEO/G";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    // Class Members
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public boolean targetVisible = false;
    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;




    //public static int[] allTrackables;

    List<VuforiaTrackable> allTrackables1 = new ArrayList<VuforiaTrackable>();


    public void hereWeGoAgain(){


     // map motors
     leftFront = hardwareMap.dcMotor.get("leftFront");
     leftBack = hardwareMap.dcMotor.get("leftBack");
     rightFront = hardwareMap.dcMotor.get("rightFront");
     rightBack = hardwareMap.dcMotor.get("rightBack");

     liftExtender = hardwareMap.dcMotor.get("liftExtender");

     liftRaise = hardwareMap.dcMotor.get("liftRaise");
     liftRaise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        runUsingEncoder();


    //map servo
        rightFGrabber = hardwareMap.servo.get("rightFGrabber");
        leftFGrabber = hardwareMap.servo.get("leftFGrabber");

        blockgrabFore = hardwareMap.servo.get("blockgrabFore");
        blockgrabAft = hardwareMap.servo.get("blockgrabAft");
    //set servo
        setFGrabber(true);


    RobotLog.i("Here we go again started (init)");

    telemetry.addData("Ah man, here we go again", "");
            telemetry.update();
    }//init, call before everything

    public void hereWeGoAuto(){







        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hardwareMap.get(BNO055IMU.class, "IMU");
        IMU.initialize(parameters);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);




        // Optional Tuning

        telemetry.addData("leftBackCounts:", leftBack.getCurrentPosition());
        telemetry.addData("rightBackCounts:", rightBack.getCurrentPosition());
        telemetry.addData("leftFrontCounts:", leftFront.getCurrentPosition());
        telemetry.addData("rightFrontCounts:", rightFront.getCurrentPosition());
        telemetry.addData("Angle:", angles.firstAngle);
        telemetry.addData("Wait for display to start before go","");
        telemetry.update();


        //reset the motors to forward/reverse
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);






        RobotLog.i("Here we go auto started(auto init)");

    }

    public void waitForGo() {
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Oh no, not this again. Do I really have to?", "");
            telemetry.update(); }
        RobotLog.i("I'm ready to go, but never enough");
    }//Call in all Autonomous OpModes



    public void driveNormalEdit(double distance){
        RobotLog.i("Starting driveNormal");

        double output = 0;

        resetEncoders();

        sleep(250);

        runUsingEncoder();

        inchesToCounts(distance);

        setTargetPosition(counts);

        if(counts > 0) {
            telemetry.addData("Current counts", counts);
            RobotLog.i("counts was above 0");
            while (opModeIsActive() && leftBack.getCurrentPosition() < leftBack.getTargetPosition() - drivingThreshold && rightBack.getCurrentPosition() < rightBack.getTargetPosition() - drivingThreshold) {
                double error = ((counts - leftBack.getCurrentPosition()));
                // error is distance from where it should be

                double a = (double) Math.abs(counts / 2);

                output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .10) / DRIVE_NORMAL_ACCEL);
                /* take error, create accel curve. add .15 so it wont try to start at 0, and divide to reduce overshoot
                 This was talked about in the programming channel of discord and I have since changed the 2 to a variable to make large changes easier.
                */
                //todo there will be much tuning here as soon as weight is added. Same with strafe

                setPowers(output * drivingPower);
                leftFront.setPower(output * drivingPower);
                rightFront.setPower(output * (drivingPower+.1));
                leftBack.setPower(output * drivingPower);
                rightFront.setPower(output * (drivingPower+.1));

                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);

                telemetry.update();
            }
            RobotLog.i("above 0 while loop has been exited");
        } else {
            telemetry.addData("Current counts", counts);
            RobotLog.i("counts was not above 0");
            while (opModeIsActive() && leftBack.getCurrentPosition() > leftBack.getTargetPosition() + drivingThreshold && rightBack.getCurrentPosition() > rightBack.getTargetPosition() + drivingThreshold) {
                double error = (counts - leftBack.getCurrentPosition());

                double a = (double) Math.abs(counts / 2);

                output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .10) / DRIVE_NORMAL_ACCEL);

                setPowers(-output * drivingPower);

                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);

                telemetry.update();
            }
            RobotLog.i("below 0 while loop has been exited");
        }
        sleep(500);
        setPowers(0);
        RobotLog.i("Finished driveNormal");




    } // driveNormalEdit is made specifically for the red side backing up to the wall. It has the right side go faster

    public void driveNormal(double distance){
        RobotLog.i("Starting driveNormal");

        double output = 0;

        resetEncoders();

        sleep(250);

        runUsingEncoder();

        inchesToCounts(distance);

        setTargetPosition(counts);

        if(counts > 0) {
            telemetry.addData("Current counts", counts);
            RobotLog.i("counts was above 0");
            while (opModeIsActive() && leftBack.getCurrentPosition() < leftBack.getTargetPosition() - drivingThreshold && rightBack.getCurrentPosition() < rightBack.getTargetPosition() - drivingThreshold) {
                double error = ((counts - leftBack.getCurrentPosition()));
                // error is distance from where it should be

                double a = (double) Math.abs(counts / 2);

                output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .10) / DRIVE_NORMAL_ACCEL);
                /* take error, create accel curve. add .15 so it wont try to start at 0, and divide to reduce overshoot
                 This was talked about in the programming channel of discord and I have since changed the 2 to a variable to make large changes easier.
                */
                //todo there will be much tuning here as soon as weight is added. Same with strafe

                setPowers(output * drivingPower);

                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);

                telemetry.update();
            }
            RobotLog.i("above 0 while loop has been exited");
        } else {
            telemetry.addData("Current counts", counts);
            RobotLog.i("counts was not above 0");
            while (opModeIsActive() && leftBack.getCurrentPosition() > leftBack.getTargetPosition() + drivingThreshold && rightBack.getCurrentPosition() > rightBack.getTargetPosition() + drivingThreshold) {
                double error = (counts - leftBack.getCurrentPosition());

                double a = (double) Math.abs(counts / 2);

                output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .10) / DRIVE_NORMAL_ACCEL);

                setPowers(-output * drivingPower);

                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);

                telemetry.update();
            }
            RobotLog.i("below 0 while loop has been exited");
        }
        sleep(500);
        setPowers(0);
        RobotLog.i("Finished driveNormal");




    }




    public void driveStrafe(double distance, boolean strafeRight){
        double output = (0);
        RobotLog.i("starting drive strafe");

        resetEncoders();

        sleep(250);

        runUsingEncoder();

        inchesToCounts(distance);


        //setTargetPosition(counts); regular drive only

        if (strafeRight == true ){
        // true means we are going right, false means left
            RobotLog.i("strafe right was true");

            telemetry.addData("strafe right true", "");
            telemetry.update();





         leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
         leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
         rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
         rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

         setTargetPosition(counts);


            while (opModeIsActive() && leftBack.getCurrentPosition() < leftBack.getTargetPosition() + strafingThreshold/* && rightBack.getCurrentPosition() < rightBack.getTargetPosition() - strafingThreshold*/){
             double error = (counts - leftBack.getCurrentPosition());
                //error is distance from where left back should be


                 double a = (double) Math.abs(counts / 2);
                 output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .25) / DRIVE_STRAFE_ACCEL);
                //create drive accell curve, make it not start at 0, and divide to change power

                setPowers(output * strafingPower);

                //todo add random tripping

                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);
                telemetry.addData("strafeRight true","");

                telemetry.update();
            }



        }
        else{
            RobotLog.i("strafe right was false");

            //telemetry.addData("strafe right false", "");
            //telemetry.update();


             //reverse the correct motors.
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            setTargetPosition(counts);


            RobotLog.i(String.valueOf(leftBack.getTargetPosition()));
            RobotLog.i(String.valueOf(leftFront.getTargetPosition()));
            RobotLog.i(String.valueOf(rightBack.getTargetPosition()));
            RobotLog.i(String.valueOf(leftFront.getTargetPosition()));



            while (opModeIsActive() && leftBack.getCurrentPosition() < leftBack.getTargetPosition() + strafingThreshold /*&& rightBack.getCurrentPosition() > rightBack.getTargetPosition() - strafingThreshold*/){
                double error = (counts - leftBack.getCurrentPosition());
                //error is distance from where left back should be



                double a = (double) Math.abs(counts / 2);
                output = ((((Math.abs(Math.abs(error) - a) / -a) + 1) + .25) / DRIVE_STRAFE_ACCEL);
                //create drive accell curve, make it not start at 0, and divide to change power

                setPowers(output * strafingPower);

                //leftBack.setPower(output * strafingPower);
                //leftFront.setPower(output * strafingPower);
                //rightFront.setPower(output * strafingPower);
                //rightBack.setPower(output * strafingPower);

                //todo add random tripping
                telemetry.addData("output", output);
                telemetry.addData("power", leftBack.getPower());
                telemetry.addData("error", error);
                telemetry.addData("strafeRight false","");

                telemetry.update();
                RobotLog.i("strafe left while loop complete");
            }


        }

      setPowers(0);
        RobotLog.i("end of strafe");

     //after the strafe, reset the motors to forward
     sleep(100);
     leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
     rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
     leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
     rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

     sleep(500);
    } //go sideways! strafeRight true means go right, and vis versa.

    public void turn(double angle) {
        runWithoutEncoder();

        while (opModeIsActive() && Math.abs(robotHeading() - angle) > turningThreshold) {
            double turnVariable;

            if (robotHeading() < angle) {
                turnVariable = ((robotHeading() - angle) / 220) - .3;
                leftBack.setPower(-turningPower * turnVariable);
                leftFront.setPower(-turningPower * turnVariable);
                rightBack.setPower(turningPower * turnVariable);
                rightFront.setPower(turningPower * turnVariable);
            } else {
                turnVariable = ((robotHeading() - angle) / 220) + .3;
                leftBack.setPower(turningPower * turnVariable);
                leftFront.setPower(turningPower * turnVariable);
                rightBack.setPower(-turningPower * turnVariable);
                rightFront.setPower(-turningPower * turnVariable);
            }
            telemetry.addData("Heading:", robotHeading());
            telemetry.update();
        }
        setPowers(0);
    } //Warning: Turning may occur. Warning: Turning may not occur Warning:Turn is in wrong direction, it works

    public void turnPOWER(double angle) {
        runWithoutEncoder();

        while (opModeIsActive() && Math.abs(robotHeading() - angle) > turningThreshold) {
            double turnVariable;

            if (robotHeading() < angle) {
                turnVariable = ((robotHeading() - angle) / 220) - .3;
                leftBack.setPower(-turningPower * turnVariable);
                leftFront.setPower(-turningPower * turnVariable);
                rightBack.setPower(turningPower * turnVariable);
                rightFront.setPower(turningPower * turnVariable);
            } else {
                turnVariable = ((robotHeading() - angle) / 220) + .3;
                leftBack.setPower(turningPower * turnVariable);
                leftFront.setPower(turningPower * turnVariable);
                rightBack.setPower(-turningPower * turnVariable);
                rightFront.setPower(-turningPower * turnVariable);
            }
            telemetry.addData("Heading:", robotHeading());
            telemetry.update();
        }
        setPowers(0);
    } //Warning: Turning may occur. Warning: Turning may not occur Warning:Turn is in wrong direction, it works


    public void setTargetPosition(double counts) {
        leftBack.setTargetPosition((int) (leftBack.getCurrentPosition() + counts));
        leftFront.setTargetPosition((int) (leftFront.getCurrentPosition() + counts));
        rightBack.setTargetPosition((int) (rightBack.getCurrentPosition() + counts));
        rightFront.setTargetPosition((int) (rightFront.getCurrentPosition() + counts));
    } //Set a target position for all drivetrain motors, should only be used in regular drive


    public void runUsingEncoder() {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    } //Set all drivetrain motors to RUN_USING_ENCODER


    public void runWithoutEncoder() {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    } //Set all drivetrain motors to RUN_WITHOUT_ENCODER


    public void resetEncoders() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    } //Reset encoder counts to... guess what


    public void inchesToCounts(double distance) {
        counts = (int) (distance * ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)));
    } //Convert from inches into encoder counts


    public void setPowers(double power) {
        leftBack.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    } //Set all drivetrain motors to a certain power


    public void setFGrabber(boolean is_open){

        if (is_open == true){
            //open
            leftFGrabber.setPosition(0);
            rightFGrabber.setPosition(1);

        }else{
            //close
            leftFGrabber.setPosition(.30);
            rightFGrabber.setPosition(.70);

        }



    } //set the grabbers open/closed in one spot, //todone make all the auto use this.


    public void grabFReturn(boolean is_red){

        if (is_red == true){
            //----------RED----------

            driveNormal(-22);                   //starting fwd

            sleep(100);                      //wait for robot to stop moving

            driveStrafe(11,false);  // drive closer to the wall

            sleep(100);                     //wait for stop

            driveNormal(-13);                  //get to the foundation

            setFGrabber(false);                         //move grabber

            sleep(500);                     //wait for grabber to move

            driveNormalEdit(20);               //drive back to wall/starting point

            turn(-90);

            setFGrabber(true);                          //let go of foundation

            turn(0);

            driveStrafe(15,true); //move while in contact with wall a small amount

            driveNormal(-3);                  //move backwards go get away from wall

        }else{
            //----------BLUE----------

            driveNormal(-17);                  //move away from wall/starting fwd

            driveStrafe(13,true); //drive towards wall

            driveNormal(-13);                //go rest of the way to foundation

            setFGrabber(false);                        //move grabbers

            sleep(500);                    //wait for grabber to move

            driveNormal(45);                  // move back to wall/starting point

            setFGrabber(true);                         //let go of foundation

            driveStrafe(14,false); //move while in contact with wall a small amount

            driveNormal(-3);                //move backwards to get away from wall

        }


    }//entire sequence for



    
    public double robotHeading() {
        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



        return angles.firstAngle;
    } //Get heading(angle) of robot


    // y - forwards
    // x - side
    // c - rotation
    public void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = SCALEDPOWER;

        /*
        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));
        */
        leftFrontMecanum = leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower);
        rightFrontMecanum = rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower);
        leftBackMecanum = leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower);
        rightBackMecanum = rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower);



    }


    public void unfold(){
        // order is up, back servo down, front servo down, extend, lower

        //liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftPos = 800;                               //set target of motor for starting move up
        liftRaise.setTargetPosition((int) liftPos); //move to position
        liftRaise.setPower(.75);                    //set power for moving
        liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION); //start moving to pos
        sleep(500);                    //wait for move
        liftRaise.setPower(0.025);                  //set power to holding

        liftExtender.setPower(1);                  //make extender start moving
        sleep(100);                     // wait for extender to move
        liftExtender.setPower(0);                   //make extender stop moving

        blockgrabAft.setPosition(.5);               //set back grabber straight down
        sleep(100);                     //wait for it to move
        blockgrabFore.setPosition(0.2);               //set for grabber out flat





    }

    public void stoneGrab(){

        if (detectedSkystone == 8){
            //red left
            RobotLog.i("Block 8, or red left was detected");
            telemetry.addData("The detected block was ","8 or red left");
            telemetry.update();

            unfold();                                        //unfold

            turn(90);                                 //face blok line

            driveStrafe(11,false);       //align with red left/8 skystone

            blockgrabFore.setPosition(.2);                  //set front grabber to out flat
            blockgrabAft.setPosition(.5);                   //and back to straight down

            driveNormal(9);                         //drive to blok line

            sleep(1500);

            liftPos = 250;                                  //lower main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            sleep(400);

            blockgrabFore.setPosition(.65);                 //close claw and wait for it to close
            blockgrabAft.setPosition(.35);
            sleep(500);

            liftPos = 660;                                   //raise main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            driveNormal(-9);                       //move back to starting spot
            driveStrafe(11,true);

            turn(0);                              //face bridge

            dropBlock(false);








        }else if (detectedSkystone == 10){
            //red middle
            RobotLog.i("Block 10, or red middle was detected");
            telemetry.addData("The detected block was ","10 or red left");
            telemetry.update();

            unfold();

            turn(90);                                  //face block line

            blockgrabFore.setPosition(.2);                    //set front grabber to out flat
            blockgrabAft.setPosition(.5);                    //and back to straight down

            driveNormal(9);                          //drive to block line is we are already aligned with correct stone

            sleep(1500);

            liftPos = 250;                                     //lower main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            sleep(400);                         //wait for everything to settle

            blockgrabFore.setPosition(.65);                 //close claw and wait for it to close
            blockgrabAft.setPosition(.35);
            sleep(500);

            liftPos = 660;                                 //raise main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            driveNormal(-9);                    //move back to start point

            turn(0);

            dropBlock(false);









        }else if (detectedSkystone == 12){
            //red right
            RobotLog.i("Block 12, or red right was detected");
            telemetry.addData("The detected block was ","12 or red left");
            telemetry.update();

            unfold();                                    //unfold

            turn(90);                            //face blok line

            driveStrafe(11,true);  //align with red left/8 skystone

            blockgrabFore.setPosition(.2);               //set front grabber to out flat
            blockgrabAft.setPosition(.5);               //and back to straight down

            driveNormal(9);                   //drive to blok line

            sleep(1500);

            liftPos = 250;                              //lower main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            sleep(400);

            blockgrabFore.setPosition(.65);             //close claw and wait for it to close
            blockgrabAft.setPosition(.35);
            sleep(500);

            liftPos = 660;                             //raise main arm, code block
            liftRaise.setTargetPosition((int) liftPos);
            liftRaise.setPower(.75);
            liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            liftRaise.setPower(0.025);

            driveNormal(-9);//move back to starting spot
            driveStrafe(11,false);

            turn(0);                          //face bridge

            dropBlock(false);

        //----------BLUE BELOW THIS LINE----------//
        }else if (detectedSkystone == 7){
            //blu right
            RobotLog.i("Block 7, or blu right was detected");
            telemetry.addData("The detected block was ","7 or blu right");
            telemetry.update();

        }else if (detectedSkystone == 9){
            //blu middle
            RobotLog.i("Block 9, or blu middle was detected");
            telemetry.addData("The detected block was ","9 or blu middle");
            telemetry.update();

        }else if (detectedSkystone == 11){
            //blu left
            RobotLog.i("Block 11, or blu left was detected");
            telemetry.addData("The detected block was ","11 or blu left");
            telemetry.update();

        }else{
            //no stone set
            RobotLog.e("stoneGrab ran with no set detected skystone");
            telemetry.addData("stonegrab ran with no detected stone","1");
            telemetry.update();

        }



    }//set detectedSkystone then call this from detect point

    public void dropBlock(boolean doubleSample){
        liftPos = 250;                                   //lower main arm, code block
        liftRaise.setTargetPosition((int) liftPos);
        liftRaise.setPower(.75);
        liftRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        liftRaise.setPower(0.025);

        driveNormal(20);                        //drieve to block drop point

        blockgrabFore.setPosition(.2);                   //set front grabber to out flat
        blockgrabAft.setPosition(.5);                    //and back to straight down
    sleep(400);                             //wait for them to move

        if (doubleSample == true){
            driveNormal(20);
                                                        //todo make option to sample wall blocks

        }else{

            driveNormal(5);                     //return to line


        }




    }

}
