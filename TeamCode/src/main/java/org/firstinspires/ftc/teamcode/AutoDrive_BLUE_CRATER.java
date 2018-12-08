package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Drive : BLUE CRATER")
public class AutoDrive_BLUE_CRATER extends MyAutoDrive {

  @Override
  public void init() {
    setAlliance(AllianceColor.BLUE, AllianceSide.CRATER);
    super.init();
  }
}
