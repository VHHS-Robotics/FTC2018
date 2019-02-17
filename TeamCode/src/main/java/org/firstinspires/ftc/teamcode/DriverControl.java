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
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="DriverControl", group="Iterative Opmode")
public class DriverControl extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static MediaPlayer mediaPlayer = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private Servo releaseServo;
    private CRServo intakeServo;
    private DcMotor liftMotor;
    private Servo liftServo;
    private DcMotor armMotor;
    private DcMotor extendMotor;

    boolean upOrDown = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // servos used for collecting minerals
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        //motor and servo used for lifting and lowering robot
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftServo = hardwareMap.get(Servo.class, "liftServo");

        releaseServo.setDirection(Servo.Direction.REVERSE);
        releaseServo.setPosition(0.0);

        liftServo.setDirection(Servo.Direction.FORWARD);
        liftServo.setPosition(0.05);

        //Intake arm motor
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        extendMotor = hardwareMap.get(DcMotor.class, "extendMotor");

        //enables encoder method (Hurray for superfluous code!)
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        enableEncoders();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean switchMode = false;

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            telemetry.addData("SwitchMode", switchMode);
            telemetry.update();

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            float strafe_y = 0;
            float strafe_x = 0;
            float turn = 0;
            if (gamepad1.start){
                if (switchMode == false){
                    switchMode = true;
                    if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(this.hardwareMap.appContext, R.raw.snas);
                    mediaPlayer.seekTo(0);
                    mediaPlayer.start();
                }
                else
                    switchMode = false;
                    mediaPlayer.stop();
            }
            if (!switchMode) {
                strafe_y = -gamepad1.left_stick_y;
                strafe_x = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;
            }
            else{
                strafe_y = gamepad1.left_stick_y;
                strafe_x = -gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x;
            }
            boolean liftRobot = gamepad1.dpad_up;
            boolean dropRobot = gamepad1.dpad_down;

            leftFrontPower = Range.clip(strafe_y + strafe_x + turn, -1.0, 1.0);
            rightFrontPower = Range.clip(strafe_y - strafe_x - turn, -1.0, 1.0);
            leftBackPower = Range.clip(strafe_y - strafe_x + turn, -1.0, 1.0);
            rightBackPower = Range.clip(strafe_y + strafe_x - turn, -1.0, 1.0);

            if(gamepad1.left_stick_button){
                if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(this.hardwareMap.appContext, R.raw.ffd);
                    mediaPlayer.seekTo(0);
                    mediaPlayer.start();
            }

            //Intake servo control
            if (gamepad1.a)//intake CRServo
                intakeServo.setPower(1);

            else
                intakeServo.setPower(0);

            //Release Servo control
            if (gamepad1.b)//kick mineral out?
                releaseServo.setPosition(1);

            else
                releaseServo.setPosition(0);


            //lifting and dropping robot
            if (liftRobot && !upOrDown) {
                liftServo.setPosition(.3);
                liftMotor.setTargetPosition(11000);
                liftMotor.setPower(0.75);
                upOrDown = true;
            }
            if (dropRobot && upOrDown) {
                liftServo.setPosition(0.05);
                liftMotor.setTargetPosition(0);
                liftMotor.setPower(0.75);
                upOrDown = false;
            }
            if (Math.abs(liftMotor.getCurrentPosition()) > Math.abs(liftMotor.getTargetPosition()) && upOrDown) {
                liftMotor.setPower(0.0);
            }
            if (Math.abs(liftMotor.getCurrentPosition()) < Math.abs(liftMotor.getTargetPosition()) && !upOrDown) {
                liftMotor.setPower(0.0);
            }

            if(gamepad1.left_trigger > 0){
                armMotor.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger > 0){
                armMotor.setPower(-gamepad1.right_trigger);
            }
            else{
                armMotor.setPower(0);
            }

            //Extend Motor control
            if(gamepad1.left_bumper){
                extendMotor.setPower(1);
            }else if(gamepad1.right_bumper){
                extendMotor.setPower(-1);
            }else{
                extendMotor.setPower(0);
            }

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void enableEncoders(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void disableEncoders(){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
