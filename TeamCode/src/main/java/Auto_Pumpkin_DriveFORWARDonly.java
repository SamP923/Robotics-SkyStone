package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 12/10/2019
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: Drive FORWARD NO COLOR SENS", group="Pumpkin: RED")
public class Auto_Pumpkin_DriveFORWARDonly extends LinearOpMode{
    Hardware_MecanumTest autopumpkin = new Hardware_MecanumTest();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        movement(.5,.5,.5,.5);

        sleep(1500);

        movement(0,0,0,0);

    }
    public void movement(double LF, double LB, double RF, double RB)
    {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }
}
