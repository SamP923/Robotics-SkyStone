package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Created by Sam on 2/27/2020
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: BLUE Skystone", group="Tests")
public class Auto_Pumpkin_BlueSkystones extends LinearOpMode{
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);
        autopumpkin.rotateArmOut();
        autopumpkin.openClaw();

        waitForStart();

        //robot front facing towards foundation side

        //move backwards toward building site
        movement(-.5,-.5,-.5,-.5);
        sleep(2000);
        movement(0,0,0,0);
        sleep(100);

        //move right towards skystones, wait until it sees it
        /*
        while ( autopumpkin.stoneDistance.getDistance(DistanceUnit.CM) > 25 ){
            movement(0.3,-0.3,-0.3,0.3);
        }*/

        //move right no sensors
        movement(0.3,-0.3,-0.3,0.3);
        sleep(1000);
        movement(0,0,0,0);
        sleep(1000);

        //grab stone
        rotateMotorArmOut();
        autopumpkin.closeClaw();
        sleep(1000);
        autopumpkin.openClaw();
        sleep(1000);
        autopumpkin.closeClaw();
        sleep(1000);

        //lift stone
        rotateMotorArmWSS();

        //move towards foundation side color sensor
        /*

         */

        // move towards foundation side no sensors
        movement(.5,.5,.5,.5);
        sleep(2000);
        movement(0,0,0,0);

        //put back stone
        rotateMotorArmOut();

        //drop stone
        autopumpkin.openClaw();
        sleep(500);

        //first reset(mid)
        rotateMotorArmIn();
        autopumpkin.closeClaw();
        sleep(1000);
        autopumpkin.rotateArmIn();
        sleep(1000);

        //go park
        /* //color sensor
        while (autopumpkin.parkColorS.red() > autopumpkin.parkColorS.blue())
        {
            movement(-.5,-.5,-.5,-.5);
        }
        */

        movement(-.5,-.5,-.5,-.5);
        sleep(1500);
        movement(0,0,0,0);

        //end program
        autopumpkin.stopAllMotors();


    }

    public void movement(double LF, double LB, double RF, double RB)
    {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }

    public void rotateMotorArmIn(){
        autopumpkin.FourBarmotor.setPower(-.3);
        sleep(1600);
        autopumpkin.FourBarmotor.setPower(0);

    }

    public void rotateMotorArmWSS(){
        autopumpkin.FourBarmotor.setPower(-.5);
        sleep(1600);
        autopumpkin.FourBarmotor.setPower(0);

    }

    public void rotateMotorArmOut(){
        autopumpkin.FourBarmotor.setPower(.3);
        sleep((long) autopumpkin.getMotorArmSleep());
        autopumpkin.FourBarmotor.setPower(0);

    }
}
