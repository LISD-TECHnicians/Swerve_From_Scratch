package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightSubsystem extends SubsystemBase {
  private Alliance alliance;

  private NetworkTable networkTable;

  public LimelightSubsystem(String limelightName) {
    this.alliance = DriverStation.getAlliance();

    this.networkTable = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public double getTx() {
    return networkTable.getEntry("tx").getDouble(0);
  }

  public double getTy() {
    return networkTable.getEntry("ty").getDouble(0);
  }

  public double getTa() {
    return networkTable.getEntry("ta").getDouble(0);
  }

  public double[] getPose() {
    if (alliance == Alliance.Blue) {
      return networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
    }
    else {
      return networkTable.getEntry("botpose_wpired").getDoubleArray(new double[7]);
    }
    
  }

  @Override
  public void periodic() {
    System.out.println(getTx());

    /* for (int i = 0; i < getPose().length; i++) {
      System.out.println(getPose()[i]);
    } */
  }

  @Override
  public void simulationPeriodic() {}
}
