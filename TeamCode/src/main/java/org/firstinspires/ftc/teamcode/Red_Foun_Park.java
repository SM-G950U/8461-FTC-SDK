package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Foun_Park", group = "Autonomous")
public class Red_Foun_Park extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        grabFReturn(true);

        driveStrafeEdit(47,true); //drive to line

        setPowers(.3);

        sleep(200);

        setPowers(0);
    }
}
