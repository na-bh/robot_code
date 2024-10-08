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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot Teleop", group="Robot")

public class Teleop extends LinearOpMode {
    public static final double ARM_POWER    =  0.4;

    public static final double WRIST_POWER    =  0.4;
    public static final double CLAW_OPEN_POSITION    =  0;
    public static final double CLAW_CLOSED_POSITION    =  1;
    ElapsedTime runtime;


    public enum AWStates {
        DOWN(87),
        UP(0);

        int wristPosition;
        private AWStates(int WRIST) {
            wristPosition = WRIST;
        }


        public int getWristPosition() {
            return wristPosition;
        }
    }
    @Override
    public void runOpMode() {

        // deviceName corresponds to the name in the robot config
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftf");
        DcMotor rightFront  = hardwareMap.get(DcMotor.class, "rightf");
        DcMotor leftBack  = hardwareMap.get(DcMotor.class, "leftb");
        DcMotor rightBack  = hardwareMap.get(DcMotor.class, "rightb");
        Servo leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        Servo rightClaw = hardwareMap.get(Servo.class, "rightclaw");
        DcMotor arm  = hardwareMap.get(DcMotor.class, "arm");
        DcMotor wrist = hardwareMap.get(DcMotor.class, "wrist");
        Servo dronelauncher = hardwareMap.get(Servo.class, "dronelauncher");

        //one side is reversed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        runtime = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        int switchVar = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            // sets power to the motors
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denom;
            double backLeftPower = (y - x + rx) / denom;
            double frontRightPower = (y - x - rx) / denom;
            double backRightPower = (y + x - rx) / denom;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            //brakes


            if (gamepad1.b) {
                dronelauncher.setPosition(0);
                sleep(300);
                dronelauncher.setPosition(0.5);
            }



            if (-gamepad2.left_stick_y == 0) {
                arm.setPower(0.05);
            } else {
                arm.setPower(-gamepad2.left_stick_y / 2);
            }




            //moves arm up and down
//            if (gamepad2.dpad_up) {
//                arm.setPower(ARM_POWER);
//            } else if (gamepad2.dpad_down) {
//                arm.setPower(-ARM_POWER);
//            } else {
//                arm.setPower(0.0);
//            }

            wrist.setPower(gamepad2.right_stick_y);

            if (gamepad2.a) {
                telemetry.addData("A Pressed: ", "yes");
                wrist.setPower(0.6);
            } else if (gamepad2.b) {
                telemetry.addData("B Pressed: ", "yes");
                wrist.setPower(-0.6);
            }



            /*
            if (gamepad2.right_stick_y > 0.1) {
                    wrist.setTargetPosition(500);
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.1) {
                    wrist.setTargetPosition(-500);
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPower(0.5);
            }
//            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//                wrist.setPower(gamepad2.right_stick_y/3);
//            }
            else if (gamepad2.a) {
                wrist.setTargetPosition(82);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(0.4);
            } else if (gamepad2.b) {
                wrist.setTargetPosition(0);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(0.4);
            }else {
                wrist.setPower(0);
            }
            */


            /*
            if (gamepad2.a) {
                wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                switchVar = 1;
            } else if (gamepad2.b) {
                wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                switchVar = 2;
            } else if (gamepad2.y) {
                wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                switchVar = 3;
            } else {
                wrist.setPower(0);
                switchVar = 0;
            }

            switch(switchVar) {
                case 1:


                    int startPos = wrist.getCurrentPosition();
                    wrist.setPower(0.4);
                    while (Math.abs(startPos - wrist.getCurrentPosition()) < 68) {
                        telemetry.addData("pos", wrist.getCurrentPosition());
                        telemetry.update();
                    }
                    wrist.setPower(0);

                    int startPos = wrist.getCurrentPosition();
                    wrist.setPower(0.4);
                    while (Math.abs(startPos - wrist.getCurrentPosition()) < 50) {
                        telemetry.addData("pos", wrist.getCurrentPosition());
                        telemetry.update();
                    }
                    wrist.setPower(0);
*/


                    /*
                    wrist.setTargetPosition(50);
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPower(0.4);


                    break;
                case 2:
                    int startPos2 = wrist.getCurrentPosition();
                    wrist.setPower(-0.4);
                    while (Math.abs(startPos2 - wrist.getCurrentPosition()) < 50) {
                        telemetry.addData("pos", wrist.getCurrentPosition());
                        telemetry.update();
                    }
                    wrist.setPower(0);

                    /*
                    wrist.setTargetPosition(AWStates.UP.wristPosition);
                    wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wrist.setPower(0.6);
                    sleep(1000);

                     */
                    /*
                    runtime.reset();
                    while (runtime.seconds() < 0.5){
                        arm.setPower(0.2);
                    }

                    switchVar = 0;
                    break;
                case 3:
                    if (gamepad2.right_stick_y > 0.1) {
                        wrist.setTargetPosition(500);
                        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPower(0.5);
                    } else if (gamepad2.right_stick_y < -0.1) {
                        wrist.setTargetPosition(-500);
                        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPower(0.5);
                    }
                    else {
                        wrist.setPower(0);
                    }
                    break;
                case 0:
                    wrist.setPower(0);
                    break;

            }
 */






            /*
            if (gamepad2.dpad_up) {
                wrist.setTargetPosition(500);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(0.5);
            }
            if (gamepad2.dpad_down) {
                wrist.setTargetPosition(-500);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(0.5);
            }


            if ((!gamepad2.dpad_up && !gamepad2.dpad_down) && (previousGamepad2.dpad_up || previousGamepad2.dpad_down)) {
                wrist.setTargetPosition(-2000);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPower(0.5);
            }

             */




            //moves claws open and closed, open and closed claw values to be changed, servo to be programmed
            if (gamepad2.right_trigger > 0.1) { //open
                leftClaw.setPosition(0.75);
                rightClaw.setPosition(0.25);
            } else if (gamepad2.left_trigger > 0.1) { //closed
                leftClaw.setPosition(0);
                rightClaw.setPosition(1);
            }

            if (gamepad2.right_bumper) {
                rightClaw.setPosition(0.25);
            } else if (gamepad2.left_bumper) {
                leftClaw.setPosition(0.75);
            }


            //5 second timer for hang
            if (gamepad2.x) {
                runtime.reset();
                while(runtime.seconds()<8) {
                    arm.setPower(-0.7);
                    if (gamepad2.y) {
                        break;
                    }
                }
            }

            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Switch Case Variable: ", switchVar);
            telemetry.update();





        }
    }
}
