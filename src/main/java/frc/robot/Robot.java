
package frc.robot;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;




public class Robot extends TimedRobot {

 




//Chasis

WPI_TalonSRX MOTORD1 = new WPI_TalonSRX(Motores.KMOTORD1);
WPI_TalonSRX MOTORD2 = new WPI_TalonSRX(Motores.KMOTORD2);
WPI_TalonSRX MOTORD3 = new WPI_TalonSRX(Motores.KMOTORD3);

WPI_TalonSRX MOTORI4 = new WPI_TalonSRX(Motores.KMOTORI4);
WPI_TalonSRX MOTORI5 = new WPI_TalonSRX(Motores.KMOTORI5);
WPI_TalonSRX MOTORI6 = new WPI_TalonSRX(Motores.KMOTORI6);

  MotorControllerGroup motsI = new MotorControllerGroup(MOTORD1, MOTORD2, MOTORD3);
  MotorControllerGroup motsD = new MotorControllerGroup(MOTORI4, MOTORI5, MOTORI6);

  DifferentialDrive chasis = new DifferentialDrive(motsI, motsD);


//Neumática 


Compressor COMPRESOR = new Compressor(0, PneumaticsModuleType.CTREPCM);

Solenoid PISTINTAKE = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTINTAKE);
Solenoid PISTCHASIS = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS);


//CONTROLES

Joystick JoystickDriver1 = new Joystick (Controllers.kJoystickDriver1);
//Joystick JoystickDriver2 = new Joystick (Controllers.KJoystickDriver2);

double velocidad;


  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {

    

  }

  @Override
  public void autonomousInit() {
   
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

  //Movimiento del chasis con control Xbox
  velocidad=JoystickDriver1.getRawAxis(3)-JoystickDriver1.getRawAxis(2); 
  chasis.arcadeDrive(velocidad, JoystickDriver1.getRawAxis(0));



  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
