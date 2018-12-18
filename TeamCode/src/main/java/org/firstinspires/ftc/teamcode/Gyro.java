package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro
{
  // The IMU sensor object
  private BNO055IMU imu;
  // State used for updating telemetry
  Orientation angles;
  // offset heading from real one
  float offsetHeading = 0.0f;

  public void init(HardwareMap hardwareMap) {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.loggingEnabled      = false;

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);
  }

  public void update(Telemetry telemetry) {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    float realHeading = getHeading();
    telemetry.addData("Gyro","Headings internal %.1f offset %.1f real %.1f",
        angles.firstAngle, offsetHeading, realHeading);
  }

  public float getHeading() {
    float curHeading360 = convertAngle360(angles.firstAngle);
    return convertAngle180(curHeading360 + offsetHeading);
  }

  public void updateHeadingOffset(float realHeading) {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    float curHeading360 = convertAngle360(angles.firstAngle);
    float realHeading360 = convertAngle360(realHeading);

    //realHeading = readHeading + offsetHeading
    //offsetHeading = realHeading - readHeading
    offsetHeading = realHeading360 - curHeading360;
  }

  public static float convertAngle360(float angle180) {
    float angle360 = angle180;
    while (angle360 < 0.0f) { angle360 += 360.0f; };
    return (angle360 % 360.0f);
  }

  public static float convertAngle180(float angle360) {
    angle360 = convertAngle360(angle360);
    if (angle360 >= 180.0f) {
      return (angle360 - 360.0f);
    } else {
      return angle360;
    }
  }
}
