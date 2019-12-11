package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class motorFunctionsTest extends LinearOpMode {

    //defines the motors
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor rightIntake;
    public DcMotor leftIntake;
    public DcMotor joint;

    @Override
    public void runOpMode(){

        //hardware maps the motors
        leftFront = hardwareMap.get(DcMotor.class, "lF");
        leftBack = hardwareMap.get(DcMotor.class, "lB");
        rightFront = hardwareMap.get(DcMotor.class, "rF");
        rightBack = hardwareMap.get(DcMotor.class, "rB");
        rightIntake = hardwareMap.get(DcMotor.class, "rI");
        leftIntake = hardwareMap.get(DcMotor.class, "lI");
        joint = hardwareMap.get(DcMotor.class, "joint");

        //reverses directions of the motors so everything runs intuitively
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //waits for the opmode to start
        waitForStart();

        while(opModeIsActive()){
            straight(24, .5);
            sleep(3000);
            turn(12, .2);
            sleep(3000);
            strafe(24, 0.2);
            sleep(3000);
            break;
        }
    }

    /**
     * runs a motor with encoder (assumes motor is a neverrest 20)
     * @param motor literally dc motor, make sure you hardware map
     * @param position in inches
     * @param power -1 to 1
     */
    public void runMotorWithEncoder(DcMotor motor, int position, double power){

        position = (int) ((position * 537.6) / (4 * 3.14159)); //converts inches to ticks
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);

    }

    /**
     *  goes straight using encoder
     * @param position use negative inches for backward, positive for forward
     * @param power always positive
     */
    public void straight(int position, double power){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, position, power);
        runMotorWithEncoder(rightFront, position, power);
        runMotorWithEncoder(rightBack, position, power);

    }

    /**
     * strafing w encoder
     * the measurements on this are most definitely sketchy but like
     * @param position in inches, left / right TBD although i think + is right
     * @param power always positive
     */
    public void strafe(int position, double power){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, -position, -power);
        runMotorWithEncoder(rightFront, -position, -power);
        runMotorWithEncoder(rightBack, position, power);

    }

    /**
     * turn w encoder (no, you can't input degrees)
     * @param position units are "inches" but that doesnt mean anything, + to go right
     * @param power always positive
     */
    public void turn(int position, double power){

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, position, power);
        runMotorWithEncoder(rightFront, -position, -power);
        runMotorWithEncoder(rightBack, -position, -power);
    }
}


