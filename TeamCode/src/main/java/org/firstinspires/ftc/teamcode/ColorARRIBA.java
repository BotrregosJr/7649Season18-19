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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="ARRIBA Color", group="Pushbot")
//@Disabled
public class ColorARRIBA extends LinearOpMode {
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // neverest 40:1
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.90;
    static final double     TURN_SPEED              = 0.5;

    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;




    /* Declare OpMode members. */
    HardwareOmni         robot   = new HardwareOmni();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.lift.setPower(-0.8);
        sleep(2700);
        robot.lift.setPower(0);
        sleep(1000);
        encoderDrive(DRIVE_SPEED,3,-3,-3,3,5.0);// slide
        sleep(1000);
        encoderDrive(DRIVE_SPEED,  -10,  -10, -10,-10,5.0); // arrasar
        sleep(1000
        );
        Color.RGBToHSV((int) (robot.color1.red() * SCALE_FACTOR),
                (int) (robot.color1.green() * SCALE_FACTOR),
                (int) (robot.color1.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance1 (cm)",
                String.format(Locale.US, "%.02f", robot.distance1.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha1", robot.color1.alpha());
        telemetry.addData("Red1  ", robot.color1.red());
        telemetry.addData("Green1", robot.color1.green());
        telemetry.addData("Blue1 ", robot.color1.blue());
        telemetry.addData("Hue1", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        Color.RGBToHSV((int) (robot.color2.red() * SCALE_FACTOR),
                (int) (robot.color2.green() * SCALE_FACTOR),
                (int) (robot.color2.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Distance2 (cm)",
                String.format(Locale.US, "%.02f", robot.distance2.getDistance(DistanceUnit.CM)));
        telemetry.addData("Alpha2", robot.color2.alpha());
        telemetry.addData("Red2  ", robot.color2.red());
        telemetry.addData("Green2", robot.color2.green());
        telemetry.addData("Blue2 ", robot.color2.blue());
        telemetry.addData("Hue2", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        telemetry.update();

        if (robot.color1.blue()<100&&robot.color2.blue()>100){
            //gold izquierda
           telemetry.addData("Izq", "Izq");
            sleep(3000);


        } else if (robot.color1.blue()>100&&robot.color2.blue()>100){
            //gold derecha
            telemetry.addData("Der", "Der");
            sleep(3000);


        } else {
            //gold centro
            telemetry.addData("Cent", "Cent");
            sleep(3000);

        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }


    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = robot.frontRightDrive.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);


            robot.frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.backRightDrive.setTargetPosition(newBackRightTarget);
            robot.frontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.backLeftDrive.setTargetPosition(newBackLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() || robot.backRightDrive.isBusy() || robot.frontRightDrive.isBusy() || robot.backLeftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget );
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d :%7d",
                        robot.frontRightDrive.getCurrentPosition(),
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.backLeftDrive.getCurrentPosition(),
                        robot.backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.backLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            robot.frontLeftDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(150);   // optional pause after each move

            robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
