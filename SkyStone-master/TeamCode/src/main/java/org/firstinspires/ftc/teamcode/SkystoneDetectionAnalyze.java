
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="SkystoneDetectionAnalyse", group="Android Studio")
public class SkystoneDetectionAnalyze extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot CleoBot = new Robot();

    private OpenCvCamera phoneCam;
    private SkystoneDetectorCV skyStoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        float fLeftStrafe = 0;
        float fRightStrafe = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Open the connection to the camera device
        phoneCam.openCameraDevice();

        skyStoneDetector = new SkystoneDetectorCV();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        CleoBot.hardwareMap = this.hardwareMap;
        CleoBot.opMode = this;
        telemetry.addData("Status", "Initialize");
        telemetry.update();
        CleoBot.Initialize();
        //Initialize Drive Train Motors.
        CleoBot.DriveTrainMotor_Init_Encoder();
        //CleoBot.DriveTrainMotor_Reset_Encoder();
        CleoBot.DriveTrainMotor_SetRun2Pos();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Stone Position", skyStoneDetector.position);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            //telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);

            
            CleoBot.Bot_Drive(-gamepad1.left_stick_y,
                                -gamepad1.right_stick_y,
                                -gamepad1.left_stick_y,
                                -gamepad1.right_stick_y);
            fLeftStrafe = gamepad1.left_trigger;
            fRightStrafe = gamepad1.right_trigger;

            // Trigger values range from 0.0 to 1.0
            if (fLeftStrafe != 0) {
                CleoBot.Bot_Drive(-fLeftStrafe,
                        fLeftStrafe,
                        fLeftStrafe,
                        -fLeftStrafe);
                // Reset variable to read the new data in the next loop
                fLeftStrafe = 0;
            }
            if (fRightStrafe != 0) {
                CleoBot.Bot_Drive(fRightStrafe,
                        -fRightStrafe,
                        -fRightStrafe,
                        fRightStrafe);
                // Reset the variable to read the new data in the next loop
                fRightStrafe = 0;
            }
            // /************************************************/
            // Diagonol Straphing
            // /************************************************/
            // Bumper & Button values are either 1 or 0
            // Y - Diagonal Forward Right
            if (gamepad1.y == true) {
                CleoBot.Bot_Drive(1,
                        0,
                        0,
                        1);
            }
            // X - Diagonal Forward Left
            if (gamepad1.x == true) {
                CleoBot.Bot_Drive(0,
                        1,
                        1,
                        0);
            }
            // B - Diagonal Reverse Right
            if (gamepad1.b == true) {
                CleoBot.Bot_Drive(-1,
                        0,
                        0,
                        -1);

            }
            // A - Diagonal Reverse Left
            if (gamepad1.a == true) {
                CleoBot.Bot_Drive(0,
                        -1,
                        -1,
                        0);
            }

        }
    }
}
