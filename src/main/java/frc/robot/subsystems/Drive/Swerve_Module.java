package frc.robot.subsystems.Drive;

import frc.robot.Constants.Drive_Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Swerve_Module {
  private final WPI_TalonFX Drive_Motor;
  private final CANSparkMax Rotation_Motor;

  private final CANCoder Rotation_Encoder;
  private final double Angle_Offset;

  private final boolean Rotation_Encoder_Invert;

  private final SlewRateLimiter Drive_Limiter = new SlewRateLimiter(Drive_Constants.Max_Drive_Set_Acceleration);

  private final PIDController Rotation_PID = new PIDController(Drive_Constants.Motor_Rotation_P, Drive_Constants.Motor_Rotation_I, Drive_Constants.Motor_Rotation_D);

  private SwerveModuleState Current_Swerve_Module_State = new SwerveModuleState();

  public Swerve_Module(int Drive_Motor_ID, int Rotation_Motor_ID, int Rotation_Encoder_ID, double Angle_Offset, boolean Drive_Motor_Invert, boolean Rotation_Motor_Invert, boolean Rotation_Encoder_Invert) {
    // Declare Swerve Module motors
    Drive_Motor = new WPI_TalonFX(Drive_Motor_ID, "rio");
    Rotation_Motor = new CANSparkMax(Rotation_Motor_ID, MotorType.kBrushless);

    Rotation_Encoder = new CANCoder(Rotation_Encoder_ID, "rio");

    Drive_Motor.configFactoryDefault();
    Rotation_Motor.restoreFactoryDefaults();

    Rotation_Encoder.configFactoryDefault();

    Drive_Motor.configVoltageCompSaturation(Drive_Constants.Nominal_Voltage);
    Rotation_Motor.enableVoltageCompensation(Drive_Constants.Nominal_Voltage);

    Drive_Motor.setNeutralMode(NeutralMode.Coast);
    Rotation_Motor.setIdleMode(IdleMode.kBrake);

    this.Rotation_Encoder_Invert = Rotation_Encoder_Invert;

    Drive_Motor.setInverted(Drive_Motor_Invert);
    Rotation_Motor.setInverted(Rotation_Motor_Invert);

    this.Angle_Offset = Angle_Offset; // Offsets built in error from Absolute Encoder

    Rotation_PID.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double Get_Drive_Position() {
    return Drive_Motor.getSelectedSensorPosition() * Drive_Constants.Drive_Motor_Position_To_Meters; // Returns meters
  }

  public double Get_Drive_Velocity() {
    return Drive_Motor.getSelectedSensorVelocity() * Drive_Constants.Drive_Motor_Velcoity_to_Meters_Second; // Returns meters per second
  }

  public double Get_Rotation_Position() {
    return (Units.degreesToRadians(Rotation_Encoder.getAbsolutePosition()) - Angle_Offset) * (Rotation_Encoder_Invert ? -1 : 1); // Returns radians
  }

  public double Get_Rotation_Velocity() {
    return Units.degreesToRadians(Rotation_Encoder.getVelocity()) * (Rotation_Encoder_Invert ? -1 : 1); // Returns radians per second
  }

  public SwerveModuleState Get_Swerve_State() {
    Current_Swerve_Module_State.speedMetersPerSecond = Get_Drive_Velocity();
    Current_Swerve_Module_State.angle = Rotation2d.fromRadians(Get_Rotation_Position());

    return Current_Swerve_Module_State;
  }

  public void Set_Swerve_State(SwerveModuleState Swerve_Module_State) {
    Swerve_Module_State = SwerveModuleState.optimize(Swerve_Module_State, Get_Swerve_State().angle);

    double Drive_Speed = Drive_Limiter.calculate(Swerve_Module_State.speedMetersPerSecond);
    Drive_Speed = Drive_Speed / Drive_Constants.Max_Drive_Speed;

    double Rotation_Speed = Rotation_PID.calculate(Get_Rotation_Position(), Swerve_Module_State.angle.getRadians());
    Rotation_Speed = MathUtil.clamp(Rotation_Speed, -Drive_Constants.Rotation_Speed_Scale_Factor, Drive_Constants.Rotation_Speed_Scale_Factor);

    Drive_Motor.set(Drive_Speed);
    Rotation_Motor.set(Rotation_Speed);
  }
}
