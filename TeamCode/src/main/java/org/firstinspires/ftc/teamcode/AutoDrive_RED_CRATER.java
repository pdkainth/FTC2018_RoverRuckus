package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Drive : RED CRATER")
public class AutoDrive_RED_CRATER extends MyAutoDrive {

  @Override
  public void init() {
    setAlliance(AllianceColor.RED, AllianceSide.CRATER);
    super.init();
  }
}
