package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red_Depo_Park", group = "Autonomous")
public class Red_Depo_Park extends org.firstinspires.ftc.teamcode.Maincanum {

    @Override
    public void runOpMode() throws InterruptedException {

        hereWeGoAgain(); //init
        hereWeGoAuto();  //autoinit
        waitForGo();


        driveNormal(-5); //drive away from wall

        driveStrafeEdit(40,false); //drive to line

        driveNormal(5); // drive back to line



    }

}
