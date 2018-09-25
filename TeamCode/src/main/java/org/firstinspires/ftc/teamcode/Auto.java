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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoControl", group="Iterative Opmode")
public class Auto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;


    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4+(1/6);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");


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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        //Instructions for the robot
        //move(-24,0,0,0);
        move(0, 24, 0, 0);

    }

    public void move2(double angle, float distance, float rotation){
        resetEncoder();
        runWithEncoder();

        int leftFrontNew ;
        int leftBackNew;
        int rightFrontNew;
        int rigthtBackNew;

        leftFrontNew = leftFront.getCurrentPosition() + (int) (distance*Math.sin(angle-45) * COUNTS_PER_INCH);
        leftBackNew = leftBack.getCurrentPosition() + (int) (distance*Math.sin(angle+45) * COUNTS_PER_INCH);
        rightFrontNew = rightFront.getCurrentPosition() + (int) (distance*Math.sin(angle+45) * COUNTS_PER_INCH);
        rigthtBackNew = rightBack.getCurrentPosition() + (int) (distance*Math.sin(angle-45) * COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFrontNew);
        leftBack.setTargetPosition(leftBackNew);
        rightFront.setTargetPosition(rightFrontNew);
        rightBack.setTargetPosition(rigthtBackNew);

        leftFront.setPower(DRIVE_SPEED);
        rightFront.setPower(DRIVE_SPEED);
        leftBack.setPower(DRIVE_SPEED);
        rightBack.setPower(DRIVE_SPEED);

        while(leftFront.isBusy()){
            //nothing
        }
        stopMotors();
    }

    public void move(float strafeY,float strafeLeft, float strafeRight, float turn){
        resetEncoder();
        runWithEncoder();

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

            //tell motors to move to new position
            leftFront.setTargetPosition(leftFrontNew);
            leftBack.setTargetPosition(leftBackNew);
            rightFront.setTargetPosition(rightFrontNew);
            rightBack.setTargetPosition(rightBackNew);

            // Assign power to motors
            leftFront.setPower(DRIVE_SPEED);
            rightFront.setPower(DRIVE_SPEED);
            leftBack.setPower(DRIVE_SPEED);
            rightBack.setPower(DRIVE_SPEED);
        }
        if(strafeLeft!=0){
            leftFrontNew = leftFront.getCurrentPosition() - (int) ((1+(1/3))*strafeLeft * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() + (int) ((1+(1/3))*strafeLeft * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) ((1+(1/3))*strafeLeft * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() - (int) ((1+(1/3))*strafeLeft * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFrontNew);
            leftBack.setTargetPosition(leftBackNew);
            rightFront.setTargetPosition(rightFrontNew);
            rightBack.setTargetPosition(rightBackNew);

            leftFront.setPower(DRIVE_SPEED-0.02);
            rightFront.setPower(DRIVE_SPEED-0.02);
            leftBack.setPower(DRIVE_SPEED-0.02);
            rightBack.setPower(DRIVE_SPEED);

        }
        if(strafeRight!=0){
            leftFrontNew = leftFront.getCurrentPosition() + (int) (strafeRight * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) (strafeRight * COUNTS_PER_INCH); //multiplying by -1 to spin motor counter-clockwise
            rightFrontNew = rightFront.getCurrentPosition() - (int) (strafeRight * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (strafeRight * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFrontNew);
            leftBack.setTargetPosition(leftBackNew);
            rightFront.setTargetPosition(rightFrontNew);
            rightBack.setTargetPosition(rightBackNew);

            leftFront.setPower(DRIVE_SPEED);
            rightFront.setPower(DRIVE_SPEED);
            leftBack.setPower(DRIVE_SPEED);
            rightBack.setPower(DRIVE_SPEED);

        }

        while(leftFront.isBusy()){
            //nothing
        }

        stopMotors();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


    public void resetEncoder() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithEncoder() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
