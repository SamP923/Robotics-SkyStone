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

        /*
            starts with FRONT facing towards foundation side
        */


        //move backwards toward building site
        movement(-.5,-.5,-.5,-.5);
        sleep(1000);// CHECK THIS VALUE FOR GOING BACK FAR/CLOSE ENOUGH
        movement(0,0,0,0);
        sleep(100);

        //move right towards skystones, wait until it sees it
        SENSmoveRightTowardsStones();
        sleep(500);// pause

        //move backwards to find skystone and grab it once found
        while ( autopumpkin.isSkystone() ){
            movement(-0.3,-0.3,-0.3,-0.3);
        }
        sleep(200);// CHECK THIS VALUE FOR MIDDLE OF STONE
        grabStone();

        //move towards foundation side, drop the stone, reset positions
        SENSmoveTowardsFoundation();
        dropStone();

        //go park next to bridge
        while ( autopumpkin.parkBlue())
        {
            movement(-.5,-.5,-.5,-.5);
        }


        movement(0,0,0,0);
        //end program and ensure stop
        autopumpkin.stopAllMotors();


    }

    public void grabStone(){
        //grab stone
        rotateMotorArmOut();
        autopumpkin.closeClaw();
        sleep(1000);

        //lift stone
        rotateMotorArmWSS();
    }

    public void dropStone(){
        //put back stone
        rotateMotorArmOut();

        //drop stone
        autopumpkin.openClaw();
        sleep(500);

        //first reset
        rotateMotorArmIn();
        autopumpkin.closeClaw();
        sleep(750);
        autopumpkin.rotateArmIn();
        sleep(750);
    }

    public void SENSmoveRightTowardsStones(){
        // CHECK THIS FOR DISTANCE UNIT
        while ( autopumpkin.stoneDistance.getDistance(DistanceUnit.CM) > 25 ){
            movement(0.3,-0.3,-0.3,0.3);
        }
        movement(0,0,0,0);
    }

    public void NSmoveRightTowardsStones(){
        movement(0.3,-0.3,-0.3,0.3);
        sleep(1000);// CHECK THIS FOR MOVEMENT TOWARDS STONES
        movement(0,0,0,0);
    }
    public void SENSmoveTowardsFoundation(){
        //looks for tape to go over
        while (autopumpkin.parkBlue()){
            movement(.5,.5,.5,.5);
        }
        sleep(500);// CHECK THIS VALUE TO MAKE SURE IT MOVES PAST THE BRIDGE TAPE
        movement(0,0,0,0);
    }

    public void NSmoveTowardsFoundation(){
        movement(.5,.5,.5,.5);
        sleep(2000);// CHECK THIS VALUE TO MAKE SURE IT MOVES PAST THE BRIDGE TAPE
        movement(0,0,0,0);
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
