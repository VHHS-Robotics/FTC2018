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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Roman is my extraterrestrial cat -Luis

@Autonomous(name="AutoControl", group="Iterative Opmode")
public class Auto extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor;
    private Servo liftServo;


    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: Andymark Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_60 = 420;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4+(1/6);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(0.0975);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();
        boolean runOnce = true;

        while (opModeIsActive()&& runOnce){
            enableEncoders();



            //Instructions for the robot
            dropAuto();//bring down
            move(0,3,0);//moves off hook
            liftAuto();//lowers lift arm

            //check mineral colors and move


            //move to depot

            //move to crater

            //Make sure this code does not repeat
            runOnce = false;
        }
    }

    public void move(float strafeY,float strafeX, float turn){
        int leftFrontNew;
        int leftBackNew;
        int rightFrontNew;
        int rightBackNew;

        if(strafeY!=0){
            //adding the distance to move in inches to current position
            leftFrontNew = leftFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if(strafeX!=0){
            leftFrontNew = leftFront.getCurrentPosition() + (int) (1.25*strafeX * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) (1.25*strafeX * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() - (int) (1.25*strafeX * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (1.25*strafeX * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if(turn!=0){
            leftFrontNew = leftFront.getCurrentPosition() - (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH) ;
            leftBackNew = leftBack.getCurrentPosition() - (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) Math.round((Math.PI*12.5*turn)/180*COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }



        while(leftFront.isBusy()) {
            telemetry.addData("LeftFontPosition", leftFront.getCurrentPosition());
            telemetry.addData("leftBackPosition", leftBack.getCurrentPosition());
            telemetry.addData("RightFontPosition", rightFront.getCurrentPosition());
            telemetry.addData("rightBackPosition", rightBack.getCurrentPosition());
            telemetry.update();
            Thread.yield();
        }

        stopMotors();
    }
    public void movePos(int leftFrontNew, int leftBackNew, int rightFrontNew, int rightBackNew){
        leftFront.setTargetPosition(leftFrontNew);
        leftBack.setTargetPosition(leftBackNew);
        rightFront.setTargetPosition(rightFrontNew);
        rightBack.setTargetPosition(rightBackNew);

        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);
    }

    public void dropAuto(){
            liftServo.setPosition(.23);
            liftMotor.setTargetPosition(11000);
            liftMotor.setPower(1);
            while (liftMotor.isBusy()) {
                telemetry.addData("Lift", "Up");
                telemetry.update();
                Thread.yield();
            }
            liftMotor.setPower(0.0);
    }
    public void liftAuto(){
        liftServo.setPosition(0.0975);
        liftMotor.setTargetPosition(0);
        liftMotor.setPower(1);
        while (liftMotor.isBusy()) {
            telemetry.addData("Lift", "Down");
            telemetry.update();
            Thread.yield();
        }
        liftMotor.setPower(0.0);
    }


    public void enableEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stopMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
