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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="GoldAlign Testing", group="DogeCV")
public class GoldAlignTesting extends LinearOpMode
{
    // Detector object
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;

    //Motor testing statements
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    int leftFrontNew;
    int leftBackNew;
    int rightFrontNew;
    int rightBackNew;


    static final double COUNTS_PER_MOTOR_REV = 280;    // eg: Andymark Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_60 = 420;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 4+(1/6);     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;


    @Override
    public void runOpMode() {
        //mapping for movement
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


        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100
        ; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 50; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!

        enableEncoders(); //enable the encoders

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            //after dropping
            move(20, 0, 0);
            move(0, 15, 0);
            if (detector.isFound()){
                //If we can see the gold after dropping
                while(!detector.getAligned()){
                    if (detector.getYPosistion() < detector.getCenter()){
                        //move left
                        telemetry.addData("Move Left", (true));
                        telemetry.update();
                        move(0, -2, 0);
                    }
                    else{
                        //move right
                        telemetry.addData("Move Right",(true)); //move foward a smudge before going right to avoid hitting lander
                        telemetry.update();
                        move(0 ,2, 0);
                    }
                }
                //now we are aligned. Go hit it.
                telemetry.addData("Move Foward", (true));
                telemetry.update();
                move(25, 0, 0);
                detector.disable();
                break;
            }
            else{//we know it is the third mineral
                //go to it and hit it
                telemetry.addData("Move to the third mineral",(true));
                telemetry.update();
                move(0, 14, 0);
                move(25, 0, 0);
                detector.disable();
                break;
            }
        }
    }
    public void move(float strafeY,float strafeX, float turn) {
        int leftFrontNew;
        int leftBackNew;
        int rightFrontNew;
        int rightBackNew;

        if (strafeY != 0) {
            //adding the distance to move in inches to current position
            leftFrontNew = leftFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (strafeY * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if (strafeX != 0) {
            leftFrontNew = leftFront.getCurrentPosition() + (int) (1.25 * strafeX * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) (1.25 * strafeX * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() - (int) (1.25 * strafeX * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) (1.25 * strafeX * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }
        if (turn != 0) {
            leftFrontNew = leftFront.getCurrentPosition() - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            leftBackNew = leftBack.getCurrentPosition() - (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightFrontNew = rightFront.getCurrentPosition() + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);
            rightBackNew = rightBack.getCurrentPosition() + (int) Math.round((Math.PI * 12.5 * turn) / 180 * COUNTS_PER_INCH);

            movePos(leftFrontNew, leftBackNew, rightFrontNew, rightBackNew);
        }


        while (leftFront.isBusy()) {
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
    public void stopMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
    public void enableEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
