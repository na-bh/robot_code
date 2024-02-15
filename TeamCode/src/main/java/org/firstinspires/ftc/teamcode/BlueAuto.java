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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Encoder Blue Auto", group="Robot")
public class BlueAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();
    DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftf");
    DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightf");
    DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftb");
    DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightb");
    Servo leftClaw = hardwareMap.get(Servo.class, "leftclaw");
    Servo rightClaw = hardwareMap.get(Servo.class, "rightclaw");
    DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
    DcMotor wrist = hardwareMap.get(DcMotor.class, "wrist");



    @Override
    public void runOpMode() {


        //one side is reversed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClaw.setPosition(0);
        rightClaw.setPosition(1);

        arm.setPower(0.4);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        arm.setPower(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        int startPos = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (gamepad1.a) {
            left();
        } else if (gamepad1.x) {
            right();
        } else if (gamepad1.b) {
            center();
        }




    }
    public void left() {
        int startPos;
        // Straight
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 1300) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 600) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(500);

        // Forward A Little Bit To Push The Pixel Onto Spike (Tape)
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 300) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Back A Little To Get Away From Spike
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 300) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 750) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Back
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 600) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 700) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 1200) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 750) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 700) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 750) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 800) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(1000);


        // Raise Arm
        startPos = arm.getCurrentPosition();
        arm.setPower(0.5);
        while (Math.abs(startPos - arm.getCurrentPosition()) < 2500) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);
        sleep(1000);


        // Drop Yellow Pixel
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);

        sleep(1000);

        // Lower Arm
        startPos = arm.getCurrentPosition();
        arm.setPower(-0.5);
        while (Math.abs(startPos - arm.getCurrentPosition()) < 2500) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);

        // Back
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 200) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);



        sleep(1000);
        // Strafe
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 1400) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward To Park
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 400) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void right() {
        int startPos;
        // Straight
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 1100) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);


        // Turn
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 900) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward A Little Bit To Push The Pixel Onto Spike (Tape)
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftBack.getCurrentPosition()) < 300) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        // Go To Backdrop
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 2050) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Raise Arm
        startPos = arm.getCurrentPosition();
        arm.setPower(0.5);
        while (Math.abs(startPos - arm.getCurrentPosition()) < 1800) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);

        // Drop Yellow Pixel
        leftClaw.setPosition(1);
        rightClaw.setPosition(0);

        sleep(1000);

        // Raise Arm
        startPos = arm.getCurrentPosition();
        arm.setPower(-0.5);
        while (Math.abs(startPos - arm.getCurrentPosition()) < 1800) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);

        sleep(500);
        // Shafe
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 1200) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Forward To Park
        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos - leftFront.getCurrentPosition()) < 300) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void center() {
        int startPos;
        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos-leftBack.getCurrentPosition()) < target ) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(300);

        startPos = leftBack.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos-leftBack.getCurrentPosition()) < 350 ) {
            telemetry.addData("pos", leftBack.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(300);




        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 800) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(300);



        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 1500) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(300);


        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 100) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(200);

        startPos = arm.getCurrentPosition();
        arm.setPower(0.5);
        while (Math.abs(startPos-arm.getCurrentPosition()) < 2500) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);
        //    sleep(3000);

        leftClaw.setPosition(1);
        rightClaw.setPosition(0);

        sleep(2000);

        startPos = arm.getCurrentPosition();
        arm.setPower(-0.5);
        while (Math.abs(startPos-arm.getCurrentPosition()) < 2500) {
            telemetry.addData("pos", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);

        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 100) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 100) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        startPos = rightFront.getCurrentPosition();
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(0.5);
        while (Math.abs(startPos-rightFront.getCurrentPosition()) < 800) {
            telemetry.addData("pos", rightFront.getCurrentPosition());
            telemetry.update();
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        startPos = leftFront.getCurrentPosition();
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);
        while (Math.abs(startPos-leftFront.getCurrentPosition()) < 400) {
            telemetry.addData("pos", leftFront.getCurrentPosition());
            telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftClaw.setPosition(0);
        rightClaw.setPosition(1);
        sleep(100);
    }
}
