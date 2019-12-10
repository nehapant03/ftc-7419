package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Motor Fxns Test", group="team7419")

public class motorFunctionsTest extends LinearOpMode {

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor rightIntake;
    public DcMotor leftIntake;
    public DcMotor joint;

    @Override
    public void runOpMode(){

        leftFront = hardwareMap.get(DcMotor.class, "lF");
        leftBack = hardwareMap.get(DcMotor.class, "lB");
        rightFront = hardwareMap.get(DcMotor.class, "rF");
        rightBack = hardwareMap.get(DcMotor.class, "rB");
        rightIntake = hardwareMap.get(DcMotor.class, "rI");
        leftIntake = hardwareMap.get(DcMotor.class, "lI");
        joint = hardwareMap.get(DcMotor.class, "joint");

        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()){
            runMotorWithEncoder(leftFront, 1120, .5);
        }
    }

    /**
     *
     * @param motor literally dc motor, make sure you hardware map
     * @param position in inches
     * @param power -1 to 1
     */
    public void runMotorWithEncoder(DcMotor motor, int position, double power){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        position = (int) (position * 1120 / (4 * 3.14159)); //converts inches to ticks
        telemetry.update();
        //position += motor.getCurrentPosition();
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     *
     * @param position use negative inches for backward, positive for forward
     * @param power always positive
     */
    public void straight(int position, double power){

        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, position, power);
        runMotorWithEncoder(rightFront, position, power);
        runMotorWithEncoder(rightBack, position, power);

        /* i hate this but i dont think we have choices? */
        while(leftBack.isBusy() && rightBack.isBusy()){}

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     * the measurements on this are most definitely sketchy but like
     * @param position in inches, left / right TBD although i think + is right
     * @param power always positive
     */
    public void strafe(int position, double power){
        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, position, -power);
        runMotorWithEncoder(rightFront, position, -power);
        runMotorWithEncoder(rightBack, position, power);

        /* i hate this but i dont think we have choices? */
        while(leftBack.isBusy() && rightBack.isBusy()){}

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    /**
     *
     * @param position units are "inches" but that doesnt mean anything, + to go right (untested)
     * @param power always positive
     */
    public void turn(int position, double power){
        runMotorWithEncoder(leftFront, position, power);
        runMotorWithEncoder(leftBack, position, power);
        runMotorWithEncoder(rightFront, position, -power);
        runMotorWithEncoder(rightBack, position, -power);

        /* i hate this but i dont think we have choices? */
        while(leftBack.isBusy() && rightBack.isBusy()){}

        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
}
