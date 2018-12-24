package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuMark_Nav {

  private static final String VUFORIA_KEY = "AWjQG/f/////AAAAGaR1lGJIDkdysS4tDu7sXQYDMWeiLDd9SoUJThu+KZdTtUifS+QbDu1xvPaSQTLtSRQuFNS82vbCdNnJZ9menqK5EWoKuH3N5BRZP14ZkIX71FS7Y1al/IzK+TEpILyoz3xoi2vPoiO1eHJpApOglq7sFPzQjrOu/12lHMI62JwzqRxuM55x++q0jgw/B3nP4duClSl4GenRihJpLA1ons/GHwtGHl3/M0cGgmQS/yYq6r/gpaNp4KCM9AdmyJ0Lstn85xnTek3EmxiLBWQ0WM16CB1zpXdo1oNlz/o8/9pZMTpTuz8zAjDkmL5W7xlGY/mT3bmaajDDWR/VZQOaRC7hL2Ic+UK4Hl97H2KCrLaW";

  // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
  // We will define some constants and conversions here
  private static final float mmPerInch        = 25.4f;
  private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

  // Data with up mount
  //private static final float CAMERA_FORWARD_POS_X  =  6.375f * mmPerInch;
  //private static final float CAMERA_VERTICAL_POS_Z =  15.5f * mmPerInch;
  //private static final float CAMERA_LEFT_POS_Y     = -8.625f * mmPerInch;
  //private static final float CAMERA_ROT_DEG_X     = (float)(90.0 + PhoneTilt.PHONE_TILT_DEG_VUFORIA);
  //private static final float CAMERA_ROT_DEG_Y     = 0.0f;

  // Data with side mount
  private static final float CAMERA_FORWARD_POS_X  =  3.75f * mmPerInch;
  private static final float CAMERA_VERTICAL_POS_Z =  10.75f * mmPerInch;
  private static final float CAMERA_LEFT_POS_Y     = -8.75f * mmPerInch;
  private static final float CAMERA_ROT_DEG_X     = (float)(90.0 + PhoneTilt.PHONE_TILT_DEG_VUFORIA);
  private static final float CAMERA_ROT_DEG_Y     = -90.0f;


  //public static final String TAG = "Vuforia VuMark for FTC";
  private OpenGLMatrix lastLocation = null;
 
  /**
   * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
   * localization engine.
   */
  private VuforiaLocalizer vuforia;
  private int cameraMonitorViewId;
  private VuforiaLocalizer.Parameters parameters;
  private VuforiaTrackables targetsRoverRuckus;
  private VuforiaTrackable blueRover;
  private VuforiaTrackable redFootprint;
  private VuforiaTrackable frontCraters;
  private VuforiaTrackable backSpace;
  List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

  boolean targetVisible;
  VuforiaTrackable visibleTarget;
  VectorF robotTranslation;
  Orientation robotRotation;


  public void init(HardwareMap hardwareMap) {
    cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
    parameters.useExtendedTracking = false;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Load the data sets that for the trackable objects. These particular data
    // sets are stored in the 'assets' part of our application.
    targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
    blueRover = targetsRoverRuckus.get(0);
    blueRover.setName("Blue-Rover");
    redFootprint = targetsRoverRuckus.get(1);
    redFootprint.setName("Red-Footprint");
    frontCraters = targetsRoverRuckus.get(2);
    frontCraters.setName("Front-Craters");
    backSpace = targetsRoverRuckus.get(3);
    backSpace.setName("Back-Space");

    // For convenience, gather together all the trackable objects in one easily-iterable collection */
    allTrackables.addAll(targetsRoverRuckus);

    // Place target on the field
    OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
        .translation(0, mmFTCFieldWidth, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
    blueRover.setLocation(blueRoverLocationOnField);

    OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
        .translation(0, -mmFTCFieldWidth, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
    redFootprint.setLocation(redFootprintLocationOnField);

    OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
        .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
    frontCraters.setLocation(frontCratersLocationOnField);

    OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
        .translation(mmFTCFieldWidth, 0, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
    backSpace.setLocation(backSpaceLocationOnField);

    OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
        .translation(CAMERA_FORWARD_POS_X, CAMERA_LEFT_POS_Y, CAMERA_VERTICAL_POS_Z)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES,
            CAMERA_ROT_DEG_X, CAMERA_ROT_DEG_Y, 0));

    //  Let all the trackable listeners know where the phone is.
    for (VuforiaTrackable trackable : allTrackables)
    {
      ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }
  }

  public void start() {
    targetsRoverRuckus.activate();
    targetVisible = false;
    visibleTarget = null;
  }

  public void scan(Telemetry telemetry) {
    targetVisible = false;
    visibleTarget = null;

    for (VuforiaTrackable trackable : allTrackables) {
      if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
        targetVisible = true;
        visibleTarget = trackable;

        lastLocation = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
        break;
      }
    }

    // Provide feedback as to where the robot is located (if we know).
    if (targetVisible) {
      // express position (translation) and rotation of robot.
      robotTranslation = lastLocation.getTranslation();
      robotRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
      telemetry.addData("Loc", "%s Pos %.1f, %.1f, %.1f Rot %.1f, %.1f, %.1f",
          visibleTarget.getName(), robotTranslation.get(0) / mmPerInch, robotTranslation.get(1) / mmPerInch, robotTranslation.get(2) / mmPerInch,
          robotRotation.firstAngle, robotRotation.secondAngle, robotRotation.thirdAngle);
    }
    else {
      telemetry.addData("Loc", "none");
    }
  }

  public boolean isTargetValid() {
    return targetVisible;
  }

  public float getPosX() {
    return (robotTranslation.get(0) / mmPerInch);
  }

  public float getPosY() {
    return (robotTranslation.get(1) / mmPerInch);
  }

  public String getVisibleTarget() {
    if (visibleTarget == null) {
      return "INVALID";
    } else {
      return visibleTarget.getName();
    }
  }

  public float getHeading() {
    return Gyro.convertAngle180(robotRotation.thirdAngle + 15.0f);
  }

  public VuforiaLocalizer getVuforia() {
    return vuforia;
  }

  public void stop() {
  }
}
