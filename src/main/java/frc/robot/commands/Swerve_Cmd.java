package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.Swerve_Subsystem;
import frc.robot.Constants.Drive_Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Swerve_Cmd extends CommandBase {
  private final Swerve_Subsystem SwerveSubsystem;

  private final DoubleSupplier X_Controller; // m/s
  private final DoubleSupplier Y_Controller; // m/s
  private final DoubleSupplier Rotation_Controller; // rad/s

  private final BooleanSupplier Robot_Oriented;
  private final BooleanSupplier Rotation_Position_Control;

  private final PIDController Rotation_Position_PID = new PIDController(Drive_Constants.Rotation_Position_Control_P, Drive_Constants.Rotation_Position_Control_I, Drive_Constants.Rotation_Position_Control_D);

  private ChassisSpeeds Swerve_Speeds = new ChassisSpeeds(); 

  public Swerve_Cmd(Swerve_Subsystem SwerveSubsystem, DoubleSupplier X_Controller, DoubleSupplier Y_Controller, DoubleSupplier Rotation_Controller, BooleanSupplier Robot_Oriented, BooleanSupplier Rotation_Position_Control) {
    this.SwerveSubsystem = SwerveSubsystem;
    this.X_Controller = X_Controller;
    this.Y_Controller = Y_Controller;
    this.Rotation_Controller = Rotation_Controller;

    this.Robot_Oriented = Robot_Oriented;
    this.Rotation_Position_Control = Rotation_Position_Control;

    Rotation_Position_PID.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double X_Speed = X_Controller.getAsDouble() * Drive_Constants.Max_Drive_Speed;
    double Y_Speed = Y_Controller.getAsDouble() * Drive_Constants.Max_Drive_Speed;

    double Rotation_Speed = Rotation_Controller.getAsDouble() * Drive_Constants.Max_Set_Rotation_Speed;
    double Rotation_Position = -Rotation_Controller.getAsDouble() * Math.PI;

    if (Robot_Oriented.getAsBoolean()) {
      Swerve_Speeds.vxMetersPerSecond = X_Speed;
      Swerve_Speeds.vyMetersPerSecond = Y_Speed;
      Swerve_Speeds.omegaRadiansPerSecond = Rotation_Speed;
    }
    else {
      if (Rotation_Position_Control.getAsBoolean()) {
        Swerve_Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          X_Speed, 
          Y_Speed, 
          MathUtil.clamp(Rotation_Position_PID.calculate(SwerveSubsystem.Get_Yaw(), Rotation_Position), -Drive_Constants.Max_Set_Rotation_Speed, Drive_Constants.Max_Set_Rotation_Speed), 
          Rotation2d.fromRadians(SwerveSubsystem.Get_Yaw()));
      }
      else {
        Swerve_Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          X_Speed, 
          Y_Speed, 
          Rotation_Speed, 
          Rotation2d.fromRadians(SwerveSubsystem.Get_Yaw()));
      }
    }

    // System.out.println(MathUtil.clamp(Rotation_Position_PID.calculate(SwerveSubsystem.Get_Yaw(), Rotation_Position), -Drive_Constants.Max_Set_Rotation_Speed, Drive_Constants.Max_Set_Rotation_Speed));
    System.out.println(Rotation_Position);

    SwerveSubsystem.Set_ChassisSpeeds(Swerve_Speeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
