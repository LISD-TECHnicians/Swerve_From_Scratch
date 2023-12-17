package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

import frc.robot.LimelightHelpers;

public class Limelight_Subsystem extends SubsystemBase {
  private final GenericEntry TX_Entry = RobotContainer.Robot_Status.add("TX", 0.0).getEntry();
  private final GenericEntry TY_Entry = RobotContainer.Robot_Status.add("TY", 0.0).getEntry();
  private final GenericEntry TA_Entry = RobotContainer.Robot_Status.add("TA", 0.0).getEntry();

  public Limelight_Subsystem() {}

  public double Get_TX() {
    return LimelightHelpers.getTX("LL1");
  }

  public double Get_TY() {
    return LimelightHelpers.getTY("LL1");
  }

  public double Get_TA() {
    return LimelightHelpers.getTA("LL1");
  }

  @Override
  public void periodic() {
    TX_Entry.setDouble(Get_TX());
    TY_Entry.setDouble(Get_TY());
    TA_Entry.setDouble(Get_TA());
  }

  @Override
  public void simulationPeriodic() {}
}
