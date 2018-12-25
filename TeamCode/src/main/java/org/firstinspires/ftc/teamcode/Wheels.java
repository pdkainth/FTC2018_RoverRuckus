package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wheels {

  public static final double AUTO_DR_POW  = 1.0;
  public static final double AUTO_ROT_POW = 0.8;

  /**
   * Motor 3- front left
   * Motor2 - front right
   * Motor 1 - back left
   * Motor 4 - back right
   */
  private DcMotor frontLeftDrive = null;
  private DcMotor frontRightDrive = null;
  private DcMotor backLeftDrive = null;
  private DcMotor backRightDrive = null;

  private static final double ENC_CNT_PER_INCH = 1440.0 / (2 * Math.PI * 2.0);
  private static final double STOP_ENC_DRIVE_DELTA = (0.3 * ENC_CNT_PER_INCH);
  private int targetEncPos = 0;

  public void init(HardwareMap hardwareMap) {
    frontLeftDrive = hardwareMap.get(DcMotor.class, "Motor3");
    frontRightDrive = hardwareMap.get(DcMotor.class, "Motor2");
    backLeftDrive = hardwareMap.get(DcMotor.class, "Motor1");
    backRightDrive = hardwareMap.get(DcMotor.class, "Motor0");

    frontLeftDrive.setPower(0.0);
    frontRightDrive.setPower(0.0);
    backLeftDrive.setPower(0.0);
    backRightDrive.setPower(0.0);

    frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

    frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void drive(double drive, double strafe, double rotate, Telemetry telemetry) {

    double frontLeftPower = drive + strafe + rotate;
    double backLeftPower = drive - strafe + rotate;
    double frontRightPower = drive - strafe - rotate;
    double backRightPower = drive + strafe - rotate;

    frontLeftDrive.setPower(frontLeftPower);
    frontRightDrive.setPower(frontRightPower);
    backLeftDrive.setPower(backLeftPower);
    backRightDrive.setPower(backRightPower);

    int pos = backRightDrive.getCurrentPosition();

    telemetry.addData("Wheel", "FL %.2f FR %.2f BL %.2f BR %.2f pos %d",
        frontLeftPower, frontRightPower, backLeftPower, backRightPower, pos);
  }

  private void resetEncoder() {
    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setTargetDrive(float distanceInch) {
    stop();
    resetEncoder();
    targetEncPos = (int)Math.round(Math.abs(distanceInch) * ENC_CNT_PER_INCH);
  }

  public boolean isTargetDriveDone() {
    int pos = Math.abs(backRightDrive.getCurrentPosition());
    int delta = targetEncPos - pos;
    if ((Math.abs(delta) < STOP_ENC_DRIVE_DELTA) || (pos > targetEncPos)) {
      stop();
      return true;
    } else {
      return false;
    }
  }

  public float getRemainingDistance() {
    int pos = Math.abs(backRightDrive.getCurrentPosition());
    int delta = targetEncPos - pos;
    return (Math.abs(delta) / (float)ENC_CNT_PER_INCH);
  }

  public void stop() {
    frontRightDrive.setPower(0);
    frontLeftDrive.setPower(0);
    backRightDrive.setPower(0);
    backLeftDrive.setPower(0);
  }
}
