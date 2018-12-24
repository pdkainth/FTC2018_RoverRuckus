/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class MineralDetect {
  private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
  private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
  private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

  private static final String VUFORIA_KEY = "AWjQG/f/////AAAAGaR1lGJIDkdysS4tDu7sXQYDMWeiLDd9SoUJThu+KZdTtUifS+QbDu1xvPaSQTLtSRQuFNS82vbCdNnJZ9menqK5EWoKuH3N5BRZP14ZkIX71FS7Y1al/IzK+TEpILyoz3xoi2vPoiO1eHJpApOglq7sFPzQjrOu/12lHMI62JwzqRxuM55x++q0jgw/B3nP4duClSl4GenRihJpLA1ons/GHwtGHl3/M0cGgmQS/yYq6r/gpaNp4KCM9AdmyJ0Lstn85xnTek3EmxiLBWQ0WM16CB1zpXdo1oNlz/o8/9pZMTpTuz8zAjDkmL5W7xlGY/mT3bmaajDDWR/VZQOaRC7hL2Ic+UK4Hl97H2KCrLaW";


  enum Position {
    UNKNOWN { public int indexOf(){return -1;} },
    LEFT    { public int indexOf(){return 2;} },
    CENTER  { public int indexOf(){return 1;} },
    RIGHT   { public int indexOf(){return 0;} };

    public abstract int indexOf();


  };

  private VuforiaLocalizer vuforia; //Vuforia localization engine.
  private TFObjectDetector tfod; //Tensor Flow Object Detection engine

  private int numRecognitions = 0;
  private int goldMineralX = -1;
  private int silverMineral1X = -1;
  private int silverMineral2X = -1;

  public void init(HardwareMap hardwareMap) {
    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
    initVuforia(hardwareMap);

    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod(hardwareMap);
    } else {
      tfod = null;
    }
  }

  public void init(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod(hardwareMap, vuforia);
    } else {
      tfod = null;
    }
  }

  /**
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia(HardwareMap hardwareMap) {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CameraDirection.FRONT;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
  }

  /**
   * Initialize the Tensor Flow Object Detection engine.
   */
  private void initTfod(HardwareMap hardwareMap) {
    initTfod(hardwareMap, vuforia);
  }

  private void initTfod(HardwareMap hardwareMap, VuforiaLocalizer vuforia) {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
      "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
  }

  public void start(Telemetry telemetry) {
    if (tfod != null) {
      tfod.activate();
    } else {
      telemetry.addData("MineralDetect", "ERROR Tensor flow object not valid");
    }
  }

  public Position detect(Telemetry telemetry) {
    Position goldPos = Position.UNKNOWN;

    if (tfod != null) {

      // getUpdatedRecognitions() will return null if no new information is available since
      // the last time that call was made.
      List<Recognition> updatedRecognitions = tfod.getRecognitions();
      if (updatedRecognitions != null) {
        numRecognitions = updatedRecognitions.size();

        for (Recognition recognition : updatedRecognitions) {
          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
            goldMineralX = (int) recognition.getLeft();
          } else if (silverMineral1X == -1) {
            silverMineral1X = (int) recognition.getLeft();
          } else {
            silverMineral2X = (int) recognition.getLeft();
          }
        }

        if (numRecognitions == 3) {
          if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
              goldPos = Position.LEFT;
            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
              goldPos = Position.RIGHT;
            } else {
              goldPos = Position.CENTER;
            }
          }
        }
      }
      telemetry.addData("MineralDetect", "#%d goldLoc %s pos[GSS] %d %d %d ",
          numRecognitions, goldPos.name(), goldMineralX, silverMineral1X, silverMineral2X);
    } else {
      telemetry.addData("MineralDetect", "ERROR Tensor flow object not valid");
    }

    return goldPos;
  }

  public Position detectUsingLeft2(Telemetry telemetry) {
    Position goldPos = Position.UNKNOWN;
    if (numRecognitions == 2) {
      if (goldMineralX == -1){
        goldPos = Position.RIGHT;
      } else {
        if (goldMineralX > silverMineral1X) {
          goldPos = Position.CENTER;
        } else {
          goldPos = Position.LEFT;
        }
      }
    }

    telemetry.addData("MineralDetect", "#%d goldLoc %s pos[GSS] %d %d %d ",
      numRecognitions, goldPos.name(), goldMineralX, silverMineral1X, silverMineral2X);

    return goldPos;
  }

  public void stop() {
    if (tfod != null) {
      tfod.shutdown();
    }
  }
}
