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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="DriverControl", group="Iterative Opmode")
public class DriverControl extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private CRServo rightServo;
    private CRServo leftServo;
    private DcMotor liftMotor;
    private Servo liftServo;
    private CRServo conveyorBelt;
    int loopnum = 0;

//    private DcMotor intakeExtend;
//    private DcMotor scoringExtend;
//    private DcMotor extendTilt;

    boolean upOrDown = false;

    //private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // Motor Encoder
    //private static final double     AXLE_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    //private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) / (AXLE_DIAMETER_INCHES * Math.PI);


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

        // servos used for collecting minerals
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");

        //motor and servo used for lifting and lowering robot
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftServo = hardwareMap.get(Servo.class, "liftServo");
        liftServo.setPosition(0.0975);

        conveyorBelt = hardwareMap.get(CRServo.class, "conveyorBelt");

        //enables encoder method (Hurray for superfluous code!)
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        enableEncoders();

        /*
        intakeExtend.setDirection(DcMotor.Direction.FORWARD);
        //work together
        scoringExtend.setDirection(DcMotor.Direction.FORWARD);
        extendTilt.setDirection(DcMotor.Direction.FORWARD);
        */

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        float strafe_y = -gamepad1.left_stick_y;
        float strafe_x = gamepad1.left_stick_x;
        float turn  =  gamepad1.right_stick_x;
        boolean liftTilt = gamepad1.y;
        boolean liftRobot = gamepad1.dpad_up;
        boolean dropRobot = gamepad1.dpad_down;

        leftFrontPower    = Range.clip(strafe_y+strafe_x+turn, -1.0, 1.0) ;
        rightFrontPower   = Range.clip(strafe_y-strafe_x-turn, -1.0, 1.0) ;
        leftBackPower   = Range.clip(strafe_y-strafe_x+turn, -1.0, 1.0) ;
        rightBackPower   = Range.clip(strafe_y+strafe_x-turn, -1.0, 1.0) ;


        //Intake servo control
        if(gamepad1.a)
            leftServo.setPower(1);
            rightServo.setPower(-0.75);
        if(gamepad1.b)
            leftServo.setPower(-1);
            rightServo.setPower(0.75);
        if(!gamepad1.b && !gamepad1.a)
            leftServo.setPower(0);
            rightServo.setPower(0);

        //lifting and dropping robot
        if (liftRobot && !upOrDown) {
            liftServo.setPosition(.23);
            liftMotor.setTargetPosition(11000);
            liftMotor.setPower(1);
            upOrDown = true;
        }
        if (dropRobot && upOrDown) {
            liftServo.setPosition(0.0975);
            liftMotor.setTargetPosition(0);
            liftMotor.setPower(1);
            upOrDown = false;
        }
        if(Math.abs(liftMotor.getCurrentPosition()) > Math.abs(liftMotor.getTargetPosition()) && upOrDown){
            liftMotor.setPower(0.0);
        }
        if(Math.abs(liftMotor.getCurrentPosition()) < Math.abs(liftMotor.getTargetPosition()) && !upOrDown){
            liftMotor.setPower(0.0);
        }


        //manual lift
        if (gamepad1.dpad_left){
            liftMotor.setPower(-1);
        }
        else if (gamepad1.dpad_right){
            liftMotor.setPower(1);
        }
        if (gamepad1.left_bumper)
            liftMotor.setPower(0);


        // conveyor control
        if(gamepad1.x)
            conveyorBelt.setPower(1);
        if(gamepad1.y)
            conveyorBelt.setPower(-1);
        if(!gamepad1.y && !gamepad1.x)
            conveyorBelt.setPower(0);
//        if(liftTilt){
//            ++toggleTwo;
//            if(toggleTwo % 2 == 0){
//                //encoder limit here
//                scoringExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                scoringExtend.setTargetPosition(scoringExtend.getCurrentPosition()+(int)());
//            }else if(toggleTwo % 2 != 0){
//                //here too
//        }

        // Send calculated power to wheels
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
