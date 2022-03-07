
package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //librerias
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControlarMecanismos;
import frc.robot.Constants.Controles;
import frc.robot.Constants.KPIDShooter;
import frc.robot.Constants.Kxbox;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;
import frc.robot.Constants.VelocidadChasis;
import frc.robot.Constants.statusrobot;

public class Robot extends TimedRobot {

  // CHASIS //
  WPI_TalonSRX MOTORD1ENC = new WPI_TalonSRX(Motores.Chasis.KMOTORD1);
  WPI_TalonSRX MOTORD2 = new WPI_TalonSRX(Motores.Chasis.KMOTORD2);
  WPI_TalonSRX MOTORD3 = new WPI_TalonSRX(Motores.Chasis.KMOTORD3);
  WPI_TalonSRX MOTORI4ENC = new WPI_TalonSRX(Motores.Chasis.KMOTORI4);
  WPI_TalonSRX MOTORI5 = new WPI_TalonSRX(Motores.Chasis.KMOTORI5);
  WPI_TalonSRX MOTORI6 = new WPI_TalonSRX(Motores.Chasis.KMOTORI6);
  MotorControllerGroup MOTSI = new MotorControllerGroup(MOTORD1ENC, MOTORD2, MOTORD3);
  MotorControllerGroup MOTSD = new MotorControllerGroup(MOTORI4ENC, MOTORI5, MOTORI6);
  DifferentialDrive chasis = new DifferentialDrive(MOTSI, MOTSD);
  DoubleSolenoid PISTCHASIS = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS1,Neumatica.KPISTCHASIS2);

  // Neumática // (los pistones estan en su respectivo mecanismo)
  Compressor COMPRESOR = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // CONTROLES //
  Joystick JoystickDriver1 = new Joystick(Controles.kJoystickDriver1);
  Joystick JoystickDriver2 = new Joystick(Controles.KJoystickDriver2);

  // INTAKE //
  Solenoid PISTINTAKE = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTINTAKE);
  WPI_VictorSPX MOTORINTAKE = new WPI_VictorSPX(Motores.Intake.KMOTORINTAKE);
  boolean motints = false;

  // SHOOTER //
  WPI_TalonFX MOTORSHOOTERLEFT = new WPI_TalonFX(Motores.Shooter.KMOTORSLeft);
  WPI_TalonFX MOTORSHOOTERRIGHT = new WPI_TalonFX(Motores.Shooter.KMOTORSRight);

  StringBuilder _sb = new StringBuilder(); /* String for output(PID) */

  // INDEXER //
  WPI_TalonSRX MOTORINDEXER = new WPI_TalonSRX(Motores.Indexer.KMOTORINDEXER);

  // CAPUCHA //
  WPI_TalonSRX MOTORCAPUCHA = new WPI_TalonSRX(Motores.Capucha.KMOTORCAPUCHA);

  // CLIMBER //
  WPI_TalonSRX MOTORCLIMBER = new WPI_TalonSRX(Motores.Climber.KMOTORCLIMBER);

  // LIMELIGHT //
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");




  // Navx //
  AHRS navx = new AHRS(SPI.Port.kMXP);

  // ESTRATEGIA AUTOAPUNTADO //
  double[] rpmVal = new double[7];

  /*
   *
   * SEPARACION DE INSTANCIAS
   *
   */

  @Override
  public void robotInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void robotPeriodic() {

    // IMPRIME LOS VALORES EN EL SMARTDASHBOARD
    boolean statusSmartcompr;
    double velocidadtest = MOTORSHOOTERLEFT.getSelectedSensorVelocity() / 4096 * 10 * 60 * 2;
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    statusSmartcompr = !statusrobot.compresorState;

    SmartDashboard.putBoolean("Intake", statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", statusSmartcompr);
    SmartDashboard.putBoolean("prueba compresor", statusrobot.compresorState);
    SmartDashboard.putNumber("velocidad", velocidadtest);
    SmartDashboard.putNumber("LL X Value", x);
    SmartDashboard.putNumber("LL Y Value", y);
    SmartDashboard.putNumber("LL X Area", area);

    



  }

  @Override
  public void autonomousInit() {
    reiniciarSensores();
    desactivartodo();

    rpmVal[0] = 600;
    rpmVal[1] = 1200;
    rpmVal[2] = 2500;
    rpmVal[3] = 3500;
    rpmVal[4] = 4300;
    rpmVal[5] = 5000;
    rpmVal[6] = 6500;

  }

  @Override
  public void autonomousPeriodic() { // Autonomo
    AutonomoTaxi();
  }

  @Override
  public void teleopInit() {

    falconpidConfig();
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void teleopPeriodic() { // Teleoperado

    // Chassis
    // Movimiento del chasis con control Xbox
    double velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RB) - JoystickDriver1.getRawAxis(Kxbox.AXES.LB);
    chasis.arcadeDrive(-VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
        -VelocidadChasis.velocidadX * velocidad);

    cambiosShifter();

    // Intake
    compresorbotonB();
    IntakeBotA();
    motorintake();

    // Shooter
    ShooterPID(120);

  }

  @Override
  public void disabledInit() {
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void disabledPeriodic() {
    desactivartodo();
    reiniciarSensores();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    ShootAdjust();
  }

  /*
   *
   *
   * // SEPARACION DE PERIODOS//
   *
   *
   */

  
  public void compresorbotonB() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButtonPressed(ControlarMecanismos.compresor)) {
      if (statusrobot.compresorState) {
        COMPRESOR.enableDigital();
        statusrobot.compresorState = false;
      } else {
        COMPRESOR.disable();
        statusrobot.compresorState = true;
      }
    }
  }

  public void motorintake() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario
    if (JoystickDriver1.getRawButtonPressed(Kxbox.BOTONES.X)) {
      if (motints) {
        MOTORINTAKE.set(0.5);
        motints = false;

      } else {
        MOTORINTAKE.set(0);
        motints = true;
      }
    }
  }

  public void IntakeBotA() {

    // ACCIONAMIENTO DE INTAKE CON BOTON "A"
    if (JoystickDriver1.getRawButtonPressed(ControlarMecanismos.intake)) {
      if (statusrobot.IntakeState) {
        PISTINTAKE.set(true);
        // MOTORINTAKE.set(0.4);
        statusrobot.IntakeState = false;
      } else {
        PISTINTAKE.set(false);
        // MOTORINTAKE.set(0);
        statusrobot.IntakeState = true;

      }
    }

  }

  public void desactivartodo() {

    // Desactiva totalmente todo, incluso si ya estaba desactivado antes
    chasis.arcadeDrive(0, 0);
    PISTCHASIS.set(Value.kOff);
    PISTINTAKE.set(false);

  }

  public void reiniciarSensores() {

    // Reset de sensores de encoders, navx.
    MOTORD1ENC.setSelectedSensorPosition(0);
    MOTORI4ENC.setSelectedSensorPosition(0);
    navx.reset();
    COMPRESOR.disable();
    statusrobot.IntakeState = false;
    statusrobot.compresorState = false;

  }

  public void cambiosShifter() {

    if (JoystickDriver1.getPOV() == ControlarMecanismos.shifter1) {
      PISTCHASIS.set(Value.kForward);
    }
    if (JoystickDriver1.getPOV() == ControlarMecanismos.shifter2) {
      PISTCHASIS.set(Value.kReverse);
    }
  }

  public void AutonomoTaxi() {

    double encoIzq = MOTORI4ENC.getSelectedSensorPosition();
    SmartDashboard.putNumber("Econder izquierdo", encoIzq);
    double encoDer = MOTORD1ENC.getSelectedSensorPosition();
    double testencodermenos = -1 * encoDer;
    SmartDashboard.putNumber("Econder derecho", testencodermenos);

    // encoizq

    double vuelta = encoIzq / 4096 / 4.17;
    double distanciainches = vuelta * 6.1 * Math.PI; // Units.inchesToMeters(3.2 );
    double distmeters = Units.inchesToMeters(distanciainches);

    SmartDashboard.putNumber("distancia?", distmeters);

    if (distmeters <= 3) {
      chasis.arcadeDrive(0, 0.5);
    }

    if (distmeters >= 3 && distmeters <= 2.05) {
      chasis.arcadeDrive(0, 0);
    }

    if (distmeters >= 3.1) {
      chasis.arcadeDrive(0, -0.3);
    }

    double direccionx = navx.getDisplacementX();
    double direcciony = navx.getDisplacementX();
    double angulo = navx.getAngle();

    SmartDashboard.putNumber("Coordenada x", direccionx);
    SmartDashboard.putNumber("Coordenada y", direcciony);
    SmartDashboard.putNumber("angulo", angulo);

    if (distmeters <= 3 && angulo <= 5) {
      chasis.arcadeDrive(-0.5, 0.7);
    }
    if (distmeters <= 3 && angulo >= 5) {
      chasis.arcadeDrive(0.4, 0.7);
    }

    if (distmeters <= 3 && angulo <= 5 && angulo >= -5) {
      chasis.arcadeDrive(-0, 0.7);
    }

  }

  public void falconpidConfig() { // no moverle a esto por favor👍

    /* Factory Default all hardware to prevent unexpected behaviour */
    MOTORSHOOTERLEFT.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    MOTORSHOOTERLEFT.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    MOTORSHOOTERLEFT.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.KPIDShooter.kPIDLoopIdx,
        Constants.KPIDShooter.kTimeoutMs);

    /* Config the peak and nominal outputs */
    MOTORSHOOTERLEFT.configNominalOutputForward(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configNominalOutputReverse(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configPeakOutputForward(1, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.configPeakOutputReverse(-1, Constants.KPIDShooter.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    MOTORSHOOTERLEFT.config_kF(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kF,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kP(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kP,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kI(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kI,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERLEFT.config_kD(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kD,
        Constants.KPIDShooter.kTimeoutMs);

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#

  }

  public void ShooterPID(double rpmtotal) {

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    double rpmconv = KPIDShooter.torpm * rpmtotal;
    double valor = -1 * rpmconv;// JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_derecho_eje_Y);

    SmartDashboard.putNumber("conv", rpmconv);

    double targetVelocity_UnitsPer100ms = valor * 3000 * 2048.0 / 600.0;
    MOTORSHOOTERLEFT.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
    _sb.setLength(0);
    MOTORSHOOTERRIGHT.set(TalonFXControlMode.Velocity, -targetVelocity_UnitsPer100ms);
  }

  public void ShootAdjust(){

    //Cambiar el Control que si va a hacer
    if (JoystickDriver2.getRawButton(ControlarMecanismos.limeAdjust)){
      double x = tx.getDouble(0.0);
      double errorHeading = x;
      double ajusteGiro = Constants.LimeLight.kp*x;

      chasis.tankDrive(ajusteGiro, -ajusteGiro);
    }



  }
  
}