/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="7419 Manual Mode", group="team7419")
//@Disabled
public class testManual extends LinearOpMode {

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor rightIntake;
    public DcMotor leftIntake;
    public DcMotor joint;

    public double kStraight = .7;
    public double kTurn = .7;
    public double kStrafe = .7;

    @Override
    public void runOpMode() {

        telemetry.addLine("initializing");

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

        telemetry.addLine("finished initializing");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.left_trigger != 0){
                leftIntake.setPower(gamepad1.left_trigger);
                rightIntake.setPower(-gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger != 0){
                rightIntake.setPower(gamepad1.right_trigger);
                leftIntake.setPower(-gamepad1.right_trigger);
            }
            else{
                rightIntake.setPower(0);
                leftIntake.setPower(0);
            }

            if(gamepad1.y){
                joint.setPower(.75);
            }
            else if(gamepad1.a){
                joint.setPower(-.75);
            }
            else{
                joint.setPower(0);
            }

            double straightPower = kStraight * gamepad1.left_stick_y;
            double turnPower = kTurn * gamepad1.left_stick_x;
            double strafePower = kStrafe * gamepad1.right_stick_x;

            double leftFrontPower = straightPower - turnPower - strafePower;
            double rightFrontPower = straightPower + turnPower + strafePower;
            double leftBackPower = straightPower - turnPower + strafePower;
            double rightBackPower = straightPower + turnPower - strafePower;

            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);

            telemetry.update();
        }
    }
}
