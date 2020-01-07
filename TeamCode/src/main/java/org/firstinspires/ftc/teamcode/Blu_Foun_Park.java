package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blu_Foun_Park", group = "Autonomous")
public class Blu_Foun_Park extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        grabFReturn(false);

        driveStrafe(45,false); //move to line





    }
}
