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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="SkystoneDetection_Red", group="Android Studio")
public class SkystoneDetection_Red extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot CleoBot = new Robot();
    private SkystoneDetectorCV.SkystonePosition skPosition;
    private OpenCvCamera phoneCam;
    private SkystoneDetectorCV skyStoneDetector;
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {

        webcamName = hardwareMap.get(WebcamName.class,"webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam = new OpenCvWebcam(webcamName,cameraMonitorViewId);
        //Open the connection to the camera device
        phoneCam.openCameraDevice();

        skyStoneDetector = new SkystoneDetectorCV();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);


        CleoBot.hardwareMap = this.hardwareMap;
        CleoBot.opMode = this;
        telemetry.addData("Status", "Initialize");
        telemetry.update();

        CleoBot.Initialize();
        //Initialize Drive Train Motors.
        CleoBot.DriveTrainMotor_Init_Encoder();
        CleoBot.DriveTrainMotor_Reset_Encoder();
        //CleoBot.DriveTrainMotor_SetRun2Pos();
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        skPosition = skyStoneDetector.position;
        telemetry.addData("Stone Position", skPosition);
        telemetry.addData("Stone Position", "test");
        telemetry.update();


        //while (opModeIsActive())
        {

            //telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            //telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
 //           skPosition = skyStoneDetector.position;
            telemetry.addData("Stone Position", skPosition);
            telemetry.update();

            if (skPosition == SkystoneDetectorCV.SkystonePosition.CENTER) {
                //Move Bot Forward 31 inches with the 30% power
                CleoBot.MoveBot_Forward(0.3,
                        0.3,
                        0.3,
                        0.3,
                        31,
                        75);

                CleoBot.MoveBot_Right(0.3,
                        0.3,
                        0.3,
                        0.3,
                        0,
                        325);
                CleoBot.Brick_Clamp();
                CleoBot.MoveSliderArmUp(0.8,500);
                 CleoBot.MoveBot_Back(
                        0.3,
                        0.3,
                        0.3,
                        0.3,
                        5,
                        0);  /*

                Thread.sleep(200);
                CleoBot.MoveBot_Left(0.3,
                0.3,
                0.3,
                0.3,
                0,
                50);
                CleoBot.MoveBot_Left(0.4,
                        0.4,
                        0.4,
                        0.4,
                        0,
                50);
                CleoBot.MoveBot_Left(0.5,
                0.5,
                0.5,
                0.5,
                0,
                 50);
                 CleoBot.MoveBot_Left(0.6,
                 0.6,
                 0.6,
                 0.6,
                   0,
                   50); */


                CleoBot.MoveBot_Left(0.5,
                        0.5,
                        0.5,
                        0.5,
                        0,
                        4600);

                CleoBot.MoveBot_Forward( 0.3,
                        0.3,
                        0.3,
                        0.3,
                        10,
                        100);
                CleoBot.Brick_UnClamp();
                Thread.sleep(500);

                CleoBot.MoveBot_Back(0.3,
                        0.4,
                        0.3,
                        0.4,
                        14,
                        0);

                CleoBot.MoveBot_Right(0.5,
                        0.5,
                        0.5,
                        0.5,
                        0,
                        6250);
                CleoBot.MoveSliderArmUp (-0.8,500);
                CleoBot.MoveBot_Forward(0.3,
                        0.35,
                        0.3,
                        0.35,
                        30,
                        0);
                CleoBot.Brick_Clamp();
                CleoBot.MoveSliderArmUp(1, 1000);
                CleoBot.MoveBot_Back(0.3,
                        0.3,
                        0.3,
                        0.3,
                        5,
                        0);
                CleoBot.MoveBot_Left(0.5,
                        0.5,
                        0.5,
                        0.5,
                        0,
                        7400);
                CleoBot.MoveBot_Forward(0.4,
                        0.3,
                        0.4,
                        0.3,
                        30,
                        0);
                CleoBot.Brick_UnClamp();
                Thread.sleep(500);
                CleoBot.MoveBot_Back(1,
                        1,
                        1,
                        1,
                        25,
                        0);
                CleoBot.MoveSliderArmUp(-1,500);
                CleoBot.MoveBot_Right(0.5,
                        0.5,
                        0.5,
                        0.5,
                        0,
                        3500);


            }
/*
            if(skPosition == SkystoneDetectorCV.SkystonePosition.RIGHT) {
            //Move Bot Forward 22 inches with the 30% power
                CleoBot.MoveBot_Forward(0.3,
                        0.3,
                        0.3,
                        0.3,
                        31,
                        0);

                CleoBot.MoveBot_Right(0.3,
                        0.3,
                        0.3,
                        0.3,
                        0,
                        5000);
            }

            if(skPosition == SkystoneDetectorCV.SkystonePosition.LEFT) {

                CleoBot.MoveBot_Forward(0.3,
                        0.3,
                        0.3,
                        0.3,
                        22,
                        0);
                CleoBot.MoveBot_Left(0.3,
                        0.3,
                        0.3,
                        0.3,
                        0,
                        2000);
            }*/
        }
        phoneCam.closeCameraDevice();
    }
}
