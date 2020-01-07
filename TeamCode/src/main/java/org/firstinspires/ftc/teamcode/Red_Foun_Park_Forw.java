package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Foun_Park_Forw", group = "Autonomous")
public class Red_Foun_Park_Forw extends org.firstinspires.ftc.teamcode.Maincanum {


    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto(); //autonomous init
        waitForGo(); // hmmmm, what could this do

        grabFReturn(true);

        driveStrafe(20,true); //strafe towards line

        driveNormal(-20); //go forwards out of way of other robots

        driveStrafe(20, true); // move onto line
    }
}
