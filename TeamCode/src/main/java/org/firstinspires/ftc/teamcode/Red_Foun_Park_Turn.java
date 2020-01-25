package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Foun_Park_Turn", group = "Autonomous")
@Disabled
public class Red_Foun_Park_Turn extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        driveNormal(-22);                   //starting fwd | -22

        sleep(100);                      //wait for robot to stop moving | org 100

        driveStrafe(5,false);  // drive closer to the wall | org 11

        sleep(100);                     //wait for stop | 100

        driveNormal(-13);                  //get to the foundation | -13

        setFGrabber(false);                         //move grabber

        sleep(500);                     //wait for grabber to move | 500

        driveNormalEdit(20);               //drive back to wall/starting point

        turn(-90);

        setFGrabber(true);                          //let go of foundation

        turn(0);

        driveStrafe(15,true); //move while in contact with wall a small amount

        driveNormal(-3);                  //move backwards go get away from wall


        driveStrafe(45,true); //drive to line
    }
}
