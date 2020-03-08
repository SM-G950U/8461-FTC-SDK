package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Foun_Turn_Park_Forw", group = "Autonomous")
public class Red_Foun_Turn_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto();  //autoinit
        waitForGo();


        driveNormal(-22);                   //starting fwd

        sleep(100);                      //wait for robot to stop moving

        driveStrafe(11,false);  // drive closer to the wall

        sleep(100);                     //wait for stop

        driveNormal(-7);                  //get to the foundation originally 13

        setFArm(true);                              //move arms
        setFGrabber(false);                          //move grabber

        driveStrafe(6,true);    //drive away from wall

        driveNormal(29);                    //drive towards start point

        turnAbsolute(80);                     //turn to face wall

        setFArm(false);                             //move arms
        setFGrabber(true);                          //move grabber
        sleep(100);                     //wait for move

        driveNormal(35);                   //drive to line

        driveStrafe(10,true);   //drive to skybridge






    }
}