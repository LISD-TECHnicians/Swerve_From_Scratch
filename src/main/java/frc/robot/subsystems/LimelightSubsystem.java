package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightSubsystem extends SubsystemBase {
  private Alliance alliance;

  private NetworkTable lloneTable = NetworkTableInstance.getDefault().getTable("llone");

  public LimelightSubsystem() {
    alliance = DriverStation.getAlliance();
  }

  public double getTx(NetworkTable networkTable) {
    return networkTable.getEntry("tx").getDouble(0);
  }

  public double getTy(NetworkTable networkTable) {
    return networkTable.getEntry("ty").getDouble(0);
  }

  public double getTa(NetworkTable networkTable) {
    return networkTable.getEntry("ta").getDouble(0);
  }

  public double[] getPose(NetworkTable networkTable) {
    if (alliance == Alliance.Blue) {
      return networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    }
    else {
      return networkTable.getEntry("botpose_wpired").getDoubleArray(new double[7]);
    }
    
  }

  @Override
  public void periodic() {
    System.out.println(getTx(lloneTable));
  }

  @Override
  public void simulationPeriodic() {}
}
