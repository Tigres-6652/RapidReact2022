// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Robot extends TimedRobot {


  final int kUnidadesPorRev = 2048;

  final int maxRPM = 6000;
  final double spShooter=0;
  final double spRPA=0;

  //Checar https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html?highlight=Flywheel#flywheel
  //Checar tambien https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/VelocityClosedLoop/src/main/java/frc/robot/Robot.java
  


  final TalonFXInvertType kinvertType = TalonFXInvertType.Clockwise;

  WPI_TalonFX shooter = new WPI_TalonFX(0, "rio");

  int _loops = 0;

  double pos_Rot;



  @Override
  public void robotInit() {

  shooter.configFactoryDefault();

  }

  @Override
  public void robotPeriodic() {


    double selSenPos = shooter.getSelectedSensorPosition(0);
    double pos_Rot = (double) selSenPos / kUnidadesPorRev;

    SmartDashboard.putNumber("Posicion", pos_Rot);

  }

  @Override
  public void autonomousInit() {
    shooter.configFactoryDefault();
    pos_Rot=0.0;
  }

  @Override
  public void autonomousPeriodic() {

   if(pos_Rot<150){
    shooter.set(ControlMode.PercentOutput, 0.1);
   }else{
     shooter.set(ControlMode.PercentOutput, 0);
   }


  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
