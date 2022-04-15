
package frc.robot;

//*********Librerias *************** */
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; //librerias
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Controles;
import frc.robot.Constants.KPIDShooter;
import frc.robot.Constants.Kxbox;
import frc.robot.Constants.LimeLight;
import frc.robot.Constants.Motores;
import frc.robot.Constants.Neumatica;
import frc.robot.Constants.VelocidadChasis;
import frc.robot.Constants.statusrobot;
import frc.robot.Constants.velocidadesShooter;
import frc.robot.Constants.Motores.Climber;


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
  Solenoid PISTCHASIS = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTCHASIS1);

  // Neumática // (los pistones estan en su respectivo mecanismo)
  Compressor COMPRESOR = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // CONTROLES //
  Joystick JoystickDriver1 = new Joystick(Controles.kJoystickDriver1);
  Joystick JoystickDriver2 = new Joystick(Controles.KJoystickDriver2);

  // INTAKE //
  Solenoid PISTINTAKE = new Solenoid(PneumaticsModuleType.CTREPCM, Neumatica.KPISTINTAKE1);
  WPI_VictorSPX MOTORINTAKE = new WPI_VictorSPX(Motores.Intake.KMOTORINTAKE);
  boolean motints = false;

  // SHOOTER //
  WPI_TalonFX MOTORSHOOTERLEFT = new WPI_TalonFX(Motores.Shooter.KMOTORSLeft);
  WPI_TalonFX MOTORSHOOTERRIGHT = new WPI_TalonFX(Motores.Shooter.KMOTORSRight);

  StringBuilder _sbshoot = new StringBuilder(); /* String for output(PID) */

  // INDEXER //
  WPI_VictorSPX MOTORINDEXER = new WPI_VictorSPX(Motores.Indexer.KMOTORINDEXER);
  DigitalInput limitindexer = new DigitalInput(Constants.LimitSwitches.indexer);

  // CAPUCHA //
  WPI_TalonSRX MOTORCAPUCHA = new WPI_TalonSRX(Motores.Capucha.KMOTORCAPUCHA);
  DigitalInput limitcapucha = new DigitalInput(Constants.LimitSwitches.capucha);
  boolean _lastButton1 = false;
  double targetPositionRotations;
  StringBuilder _sbcapucha = new StringBuilder(); /* String for output(PID) */

  double AnguloCapuchaConfig;

  // CLIMBER //
  CANSparkMax MOTORCLIMBER = new CANSparkMax(Climber.KMOTORCLIMBER, MotorType.kBrushless);

  // PDP
  PowerDistribution pdp = new PowerDistribution();

  // LIMELIGHT //////
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  double ajustdist;
  double ajutGi;

  // Navx /////
  AHRS navx = new AHRS(SPI.Port.kMXP);

  // ESTRATEGIA AUTOAPUNTADO //
  double ktick2Degree = 56.88;
  double capuchavalor;
  double capucha_angulo;
  double anguloFinal;

  // autonomo
  double KPgiro = 0.0055;
  double headin;
  double distmeters;
  double angulo;
  int estadoAuto = 0;
  double timeAuto = 0;

  double AutoInit;
  double time;
  double deltaMatchTime;

  // Variables solas
  double valueAuto;
  double lecturaAuto;

  double anguloo;
  double velocidaad;
  double ajusteanguloamotor;

  /*
   *
   * SEPARACION DE INSTANCIAS
   *
   */

  @Override
  public void robotInit() {
    reiniciarSensores();
    CameraServer.startAutomaticCapture();

  }

  @Override
  public void robotPeriodic() {

    resetLimitSwitch();
    time = Timer.getFPGATimestamp();
    // Cálculos
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    capuchavalor = MOTORCAPUCHA.getSelectedSensorPosition();
    anguloFinal = -1 * capuchavalor / ktick2Degree;
    double angleToGoalDegrees = LimeLight.anguloInclinacionLL + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanciaFender = (LimeLight.alturaUpperPulgadas - LimeLight.alturaAlPisoPugadasLL)
        / Math.tan(angleToGoalRadians);

    angulo = navx.getAngle();

    // IMPRIME LOS VALORES EN EL SMARTDASHBOARD

    // SmartDashboard.putBoolean("Intake", !statusrobot.IntakeState);
    SmartDashboard.putBoolean("Compresor", !statusrobot.compresorState);
     SmartDashboard.putNumber("RPM shooter",
     MOTORSHOOTERRIGHT.getSelectedSensorVelocity() * -1 * 10 * 60 / 2048);
    // SmartDashboard.putNumber("Current Shooter",
    // (pdp.getCurrent(12) + pdp.getCurrent(3)) / 2);
    // SmartDashboard.putBoolean("Reset Capucha", limitcapucha.get());
    SmartDashboard.putNumber("Valor Capucha",
        MOTORCAPUCHA.getSelectedSensorPosition());
    SmartDashboard.putNumber("Corriente Capucha",
        MOTORCAPUCHA.getSupplyCurrent());
    SmartDashboard.putNumber("Angulo Capucha", anguloFinal);
    SmartDashboard.putNumber("LL X Value", x);
    SmartDashboard.putNumber("LL Y Value", y);
    SmartDashboard.putNumber("LL X Area", area);
    SmartDashboard.putNumber("Distancia Fender", distanciaFender);
    lecturaAuto = SmartDashboard.getNumber("autoValue", 0);

    velocidaad = SmartDashboard.getNumber("ajuste velocidad", 0);
     anguloo = SmartDashboard.getNumber("ajuste ANGULO", 0);

    ///// /*NO BORRAR, SON LECTURAS EN CASO DE ALGUN ERROR DETECTARLO MAS RAPIDO*/

    // SmartDashboard.putNumber("distanciaaaaa", distmeters);
    // SmartDashboard.putNumber("ajuste GI", ajutGi);

    // SmartDashboard.putBoolean("LIMIT", limitcapucha.get());

    // SmartDashboard.putNumber("angulochasis", navx.getAngle());
    // SmartDashboard.putNumber("CAPU", MOTORCAPUCHA.getSelectedSensorPosition());

    // SmartDashboard.putNumber("estado auto", estadoAuto);
    // SmartDashboard.putNumber("tiempo delta", deltaMatchTime);

    // DISTANCIAS??
    double encoIzq = MOTORI4ENC.getSelectedSensorPosition();
    SmartDashboard.putNumber("Econder izquierdo", encoIzq);
    double encoDer = MOTORD1ENC.getSelectedSensorPosition();
    double testencodermenos = -1 * encoDer;
    SmartDashboard.putNumber("Econder derecho", testencodermenos);
    double distancia = (encoIzq + testencodermenos) / 2;
    // encoizq
    double vuelta = distancia / 4096 / 4.17;
    double distanciainches = vuelta * 6.1 * Math.PI; // Units.inchesToMeters(3.2 );
    distmeters = Units.inchesToMeters(distanciainches);



  }

  @Override
  public void autonomousInit() {
    reiniciarSensores();
    desactivartodo();
    falconpidConfig();
    capuchaPIDinit();
    estadoAuto = 0;
    // AutoInit = Timer.getFPGATimestamp();

  }

  @Override
  public void autonomousPeriodic() { //

    if (lecturaAuto == 0) {
      autonomo_2_cargos_linea_fender();


    } else if (lecturaAuto == 1) {
      autonomo_1_taxi_pegado_fender();

    } else if (lecturaAuto == 2) {

      autonomo_taxi_a_finales();
    }

  }

  @Override
  public void teleopInit() {

    falconpidConfig();
    reiniciarSensores();
    desactivartodo();
    PIDchasis();
    capuchaPIDinit();

    AnguloCapuchaConfig = 0;

  }

  @Override
  public void teleopPeriodic() { // Teleoperado

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.A)) {
      lanzamiento_de_distintos_lados();
    } else {
      // Mover Chassis
      double velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RT) - JoystickDriver1.getRawAxis(Kxbox.AXES.LT);
      chasis.arcadeDrive(
          -VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
          -VelocidadChasis.velocidadX * -velocidad);
    }
    returnHome();
    compresor();
    Intake();
    climbler();
    anguloyvelocidad();
    if (JoystickDriver1.getPOV() == Kxbox.POV.izquierda) {
      PISTCHASIS.set(false);

    } else if (JoystickDriver1.getPOV() == Kxbox.POV.derecha) {
      PISTCHASIS.set(true);
    }

    if (JoystickDriver1.getPOV() == Kxbox.POV.abajo) {

      MOTORD1ENC.setNeutralMode(NeutralMode.Brake);
      MOTORD2.setNeutralMode(NeutralMode.Brake);
      MOTORD3.setNeutralMode(NeutralMode.Brake);
      MOTORI4ENC.setNeutralMode(NeutralMode.Brake);
      MOTORI5.setNeutralMode(NeutralMode.Brake);
      MOTORI6.setNeutralMode(NeutralMode.Brake);
  
    } else {

      MOTORD1ENC.setNeutralMode(NeutralMode.Coast);
      MOTORD2.setNeutralMode(NeutralMode.Coast);
      MOTORD3.setNeutralMode(NeutralMode.Coast);
      MOTORI4ENC.setNeutralMode(NeutralMode.Coast);
      MOTORI5.setNeutralMode(NeutralMode.Coast);
      MOTORI6.setNeutralMode(NeutralMode.Coast);
    }

   /* ajustarvelocidad();
    AjustarAngulo();

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.A)) {
      lanzamiento_de_distintos_lados();
    } else {
      // Mover Chassis
      double velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RT) - JoystickDriver1.getRawAxis(Kxbox.AXES.LT);
      chasis.arcadeDrive(
          -VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
          -VelocidadChasis.velocidadX * -velocidad);
    }*/

  }

  @Override
  public void disabledInit() {
    reiniciarSensores();
    desactivartodo();
    navx.reset();

  }

  @Override
  public void disabledPeriodic() {

    desactivartodo();

  }

  @Override
  public void testInit() {
    falconpidConfig();
    capuchaPIDinit();
    reiniciarSensores();
    desactivartodo();

  }

  @Override
  public void testPeriodic() {
    ajustarvelocidad();
    AjustarAngulo();

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.A)) {
      lanzamiento_de_distintos_lados();
    } else {
      // Mover Chassis
      double velocidad = JoystickDriver1.getRawAxis(Kxbox.AXES.RT) - JoystickDriver1.getRawAxis(Kxbox.AXES.LT);
      chasis.arcadeDrive(
          -VelocidadChasis.velocidadgiro * JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_izquierdo_eje_X),
          -VelocidadChasis.velocidadX * -velocidad);
    }

  }

  /*
   *
   *
   * // SEPARACION DE PERIODOS//
   *
   *
   */

  public void compresor() {

    // SE PRENDE EL COMPRESOR CON EL BOTON "B"
    // Mas adelante cambiar esto al driver secundario

    if (JoystickDriver1.getRawButtonPressed(Kxbox.BOTONES.boton_con_lineas)) {
      if (statusrobot.compresorState) {
        COMPRESOR.enableDigital();
        statusrobot.compresorState = false;
      } else {
        COMPRESOR.disable();
        statusrobot.compresorState = true;
      }
    }
  }

  public void Intake() {

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.LB)) {

      PISTINTAKE.set(false);
      MOTORINTAKE.set(0);

    }

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.RB)) {
      PISTINTAKE.set(true);
      MOTORINTAKE.set(0.7);

    }

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.X) == true) {

      MOTORINTAKE.set(-0.35);

    }

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.B) == true) {

      MOTORINTAKE.set(0.45);

    }

    if (JoystickDriver1.getRawButton(Kxbox.BOTONES.Y) == true) {

      MOTORINTAKE.set(-0);

    }

  }

  public void desactivartodo() {

    // Desactiva totalmente todo, incluso si ya estaba desactivado antes
    chasis.arcadeDrive(0, 0);
    PISTCHASIS.set(true);
    PISTINTAKE.set(false);
    MOTORINTAKE.set(0);
    MOTORINDEXER.set(0);
    MOTORSHOOTERLEFT.set(0);
    MOTORSHOOTERRIGHT.set(0);
    MOTORCAPUCHA.set(0);
    // MOTORCLIMBER.set(0);
    MOTORCLIMBER.restoreFactoryDefaults();

  }

  public void reiniciarSensores() {

    // Reset de sensores de encoders, navx.
    MOTORD1ENC.setSelectedSensorPosition(0);
    MOTORI4ENC.setSelectedSensorPosition(0);
    COMPRESOR.disable();
    statusrobot.IntakeState = false;
    statusrobot.compresorState = false;
    velocidadesShooter.velocidad = 0;
    AnguloCapuchaConfig = 0;
    MOTORCLIMBER.restoreFactoryDefaults();
    navx.reset();

    MOTORCAPUCHA.set(0);

  }

  public void AutonomoTaxiPegadoFender() { // Se mueve :) Pa delante

    double encoIzq = MOTORI4ENC.getSelectedSensorPosition();
    SmartDashboard.putNumber("Econder izquierdo", encoIzq);
    double encoDer = MOTORD1ENC.getSelectedSensorPosition();
    double testencodermenos = -1 * encoDer;
    SmartDashboard.putNumber("Econder derecho", testencodermenos);
    double distancia = (encoIzq + testencodermenos) / 2;
    // encoizq
    double vuelta = distancia / 4096 / 4.17;
    double distanciainches = vuelta * 6.1 * Math.PI; // Units.inchesToMeters(3.2 );
    double distmeters = Units.inchesToMeters(distanciainches);

    // SmartDashboard.putNumber("distancia", distmeters);

    if (distmeters <= 2.5) {
      chasis.arcadeDrive(0, -0.7);
    }

    if (distmeters >= 2.51 && distmeters <= 2.6) {
      chasis.arcadeDrive(0, 0);
      MOTORD1ENC.setNeutralMode(NeutralMode.Brake);
      MOTORD2.setNeutralMode(NeutralMode.Brake);
      MOTORD3.setNeutralMode(NeutralMode.Brake);
      MOTORI4ENC.setNeutralMode(NeutralMode.Brake);
      MOTORI5.setNeutralMode(NeutralMode.Brake);
      MOTORI6.setNeutralMode(NeutralMode.Brake);

    }

    if (distmeters >= 2.61) {
      chasis.arcadeDrive(0, 0.4);
    }
  }

  public void AutonomoTaxi() { // Se mueve :) Pa delante

    double encoIzq = MOTORI4ENC.getSelectedSensorPosition();
    SmartDashboard.putNumber("Econder izquierdo", encoIzq);
    double encoDer = MOTORD1ENC.getSelectedSensorPosition();
    double testencodermenos = -1 * encoDer;
    SmartDashboard.putNumber("Econder derecho", testencodermenos);
    double distancia = (encoIzq + testencodermenos) / 2;
    // encoizq
    double vuelta = distancia / 4096 / 4.17;
    double distanciainches = vuelta * 6.1 * Math.PI; // Units.inchesToMeters(3.2 );
    double distmeters = Units.inchesToMeters(distanciainches);

    // SmartDashboard.putNumber("distancia?", distmeters);

    if (distmeters <= 1) {
      chasis.arcadeDrive(0, -0.7);
    }

    if (distmeters >= 1.1 && distmeters <= 1.2) {
      chasis.arcadeDrive(0, 0);
      MOTORD1ENC.setNeutralMode(NeutralMode.Brake);
      MOTORD2.setNeutralMode(NeutralMode.Brake);
      MOTORD3.setNeutralMode(NeutralMode.Brake);
      MOTORI4ENC.setNeutralMode(NeutralMode.Brake);
      MOTORI5.setNeutralMode(NeutralMode.Brake);
      MOTORI6.setNeutralMode(NeutralMode.Brake);

    }

    if (distmeters >= 1.21) {
      chasis.arcadeDrive(0, 0.4);
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

    MOTORSHOOTERLEFT.configOpenloopRamp(1.5);

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    /* Factory Default all hardware to prevent unexpected behaviour */
    MOTORSHOOTERRIGHT.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    MOTORSHOOTERRIGHT.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    MOTORSHOOTERRIGHT.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.KPIDShooter.kPIDLoopIdx,
        Constants.KPIDShooter.kTimeoutMs);

    /* Config the peak and nominal outputs */
    MOTORSHOOTERRIGHT.configNominalOutputForward(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.configNominalOutputReverse(0, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.configPeakOutputForward(1, Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.configPeakOutputReverse(-1, Constants.KPIDShooter.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    MOTORSHOOTERRIGHT.config_kF(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kF,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.config_kP(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kP,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.config_kI(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kI,
        Constants.KPIDShooter.kTimeoutMs);
    MOTORSHOOTERRIGHT.config_kD(Constants.KPIDShooter.kPIDLoopIdx, Constants.KPIDShooter.kGains_Velocit.kD,
        Constants.KPIDShooter.kTimeoutMs);

    MOTORSHOOTERRIGHT.configOpenloopRamp(2);

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#

  }

  public void ShooterPID(double rpmtotal) { // 6380 maximo

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    double rpmconv = KPIDShooter.torpm * rpmtotal;
    double valor = -1 * rpmconv;// JoystickDriver1.getRawAxis(Kxbox.AXES.joystick_derecho_eje_Y);


    double targetVelocity_UnitsPer100ms = valor * 3000 * 2048.0 / 600.0;
    _sbshoot.setLength(0);
    MOTORSHOOTERLEFT.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

    MOTORSHOOTERRIGHT.set(TalonFXControlMode.Velocity, -targetVelocity_UnitsPer100ms);
  }

  public void ajustedelanzamiento(double distacia) {

    double x = tx.getDouble(0.0);
    double ajusteGiro = 0.0f;
    float min_command = 0.03f;

    if (x > 0.1) {

      ajusteGiro = Constants.LimeLight.kp * x - min_command;

    } else if (x < 0.1) {

      ajusteGiro = Constants.LimeLight.kp * x + min_command;

    }

    // distancia
    double y = ty.getDouble(0.0);
    double angleToGoalDegrees = LimeLight.anguloInclinacionLL + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanciaFender = (LimeLight.alturaUpperPulgadas - LimeLight.alturaAlPisoPugadasLL)
        / Math.tan(angleToGoalRadians);
    double distanciaKP = -0.095;
    double DistanciaError = distacia - distanciaFender;
    double ajustedistancia = distanciaKP * DistanciaError;

    if (ajustedistancia > 0.46) {

      ajustdist = 0.46;

    } else if (ajustedistancia < 0.46&&ajustedistancia>-0.46) {

      ajustdist = ajustedistancia;

    }else if(ajustedistancia<-0.46){

      ajustdist = -0.46;

    }

    if (ajusteGiro > 0.6) {

      ajutGi = 0.6;

    } else if (ajusteGiro < 0.6&&ajusteGiro>-0.6) {

      ajutGi = ajusteGiro;

    }else if(ajusteGiro<-0.6){

      ajutGi = -0.6;

    }

    chasis.arcadeDrive(ajutGi, ajustdist);

  }

  public void ajustedegiroautonomo() { // Probar

    double x = tx.getDouble(0.0);
    double ajusteGiro = 0.0f;
    float min_command = 0.03f;

    if (x > 0.2) {

      ajusteGiro = Constants.LimeLight.kp * x - min_command;

    } else if (x < 0.2) {

      ajusteGiro = Constants.LimeLight.kp * x + min_command;

    }

    // distancia
    double y = ty.getDouble(0.0);
    double angleToGoalDegrees = LimeLight.anguloInclinacionLL + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanciaFender = (LimeLight.alturaUpperPulgadas - LimeLight.alturaAlPisoPugadasLL)
        / Math.tan(angleToGoalRadians);
    double distanciaKP = -0.065;
    double DistanciaError = 70 - distanciaFender;
    double ajustedistancia = distanciaKP * DistanciaError;

    if (ajustedistancia > 0.55) {

      ajustdist = 0.55;

    } else if (ajustedistancia < 0.55) {

      ajustdist = ajustedistancia;

    }

    if (ajusteGiro > 0.45) {

      ajutGi = 0.45;

    } else if (ajusteGiro < 0.45) {

      ajutGi = ajusteGiro;

    }

    chasis.arcadeDrive(ajutGi, ajustdist);

  }

  public void climbler() {

    if (JoystickDriver2.getPOV() == Kxbox.POV.arriba) {

      MOTORCLIMBER.set(1);
    }
    if (JoystickDriver2.getPOV() == Kxbox.POV.abajo) {

      MOTORCLIMBER.set(-1);
    } else if (JoystickDriver2.getPOV() == -1) {
      MOTORCLIMBER.set(0);
    }

  }

  public void returnHome() {

    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.RB)) {
     
        AnguloCapuchaConfig = 0;

      } 
    }

  

  public void resetLimitSwitch() {
    if (limitcapucha.get() == false) {
      MOTORCAPUCHA.setSelectedSensorPosition(0);
    }
  }

  public void capuchaPIDinit() {
    /* Factory Default all hardware to prevent unexpected behaviour */
    MOTORCAPUCHA.configFactoryDefault();

    /* Config sensor used for Primary PID [Velocity] */
    MOTORCAPUCHA.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.KPIDCapucha.kPIDLoopIdx,
        Constants.KPIDCapucha.kTimeoutMs);

    /**
     * Phase sensor accordingly.
     * Positive Sensor Reading should match Green (blinking) Leds on Talon
     */
    MOTORCAPUCHA.setSensorPhase(true);

    /* Config the peak and nominal outputs */
    MOTORCAPUCHA.configNominalOutputForward(0, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configNominalOutputReverse(0, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configPeakOutputForward(1, Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.configPeakOutputReverse(-1, Constants.KPIDCapucha.kTimeoutMs);
    /* Config the Velocity closed loop gains in slot0 */
    MOTORCAPUCHA.config_kF(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kF,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kP(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kP,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kI(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kI,
        Constants.KPIDCapucha.kTimeoutMs);
    MOTORCAPUCHA.config_kD(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kD,
        Constants.KPIDCapucha.kTimeoutMs);

    MOTORCAPUCHA.configOpenloopRamp(0.3);

  }

  public void capuchaPIDteleop(double rpmcapucha) {

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    double rpmconve = KPIDShooter.torpm * rpmcapucha;
    double valor = -1 * rpmconve;

    SmartDashboard.putNumber("conv", rpmconve);

    double targetVelocity_UnitsPer100ms = valor * 3000 * 2048.0 / 600.0;
    MOTORCAPUCHA.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    _sbshoot.setLength(0);

  }

  public void anguloyvelocidad() {

    // Apuntar
    if (JoystickDriver2.getRawAxis(Kxbox.AXES.LT) >= 0.3) {
      ShooterPID(velocidadesShooter.velocidad);
    } else {
      MOTORSHOOTERLEFT.set(0);
      MOTORSHOOTERRIGHT.set(-0);
    }

    // Disparar
    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.LB) && limitindexer.get() == true) {
      MOTORINDEXER.set(0.3);
    } else if (JoystickDriver2.getRawAxis(Kxbox.AXES.RT) >= 0.3) {
      MOTORINDEXER.set(0.3);
    } else {
      MOTORINDEXER.set(0);
    }

    // Tiro Fender
    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.B)) {
      velocidadesShooter.velocidad = velocidadesShooter.fender; // 4650
      AnguloCapuchaConfig = Constants.anguloCapucha.fender;
    }

    // Tarmac 1.84m
    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.A)) {
      AnguloCapuchaConfig = Constants.anguloCapucha.tarmac;
      velocidadesShooter.velocidad = velocidadesShooter.tarmac;
    }
    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.X)) {
      AnguloCapuchaConfig = Constants.anguloCapucha.launchpad;
      velocidadesShooter.velocidad = velocidadesShooter.launchpad;
    }


      double kpangulo=0.2;
       double anguloerror = -AnguloCapuchaConfig-anguloFinal;
       double ajusteangulo=anguloerror*kpangulo;

       if (ajusteangulo > 0.5) {

        ajusteanguloamotor = 0.5;
  
      } else if (ajusteangulo < 0.5&&ajusteangulo>-0.5) {
  
        ajusteanguloamotor = ajusteangulo;
  
      }else if(ajusteangulo<-0.5){
  
        ajusteanguloamotor = -0.5;
  
      }

      MOTORCAPUCHA.set(-ajusteanguloamotor);



    }

  

  public void ajusteDeTiroautonomo() {
    if (anguloFinal >= (-AnguloCapuchaConfig + 1)) {

      MOTORCAPUCHA.set(0.25);
    } else if ((anguloFinal >= (-AnguloCapuchaConfig - 1)) && (anguloFinal <= (-AnguloCapuchaConfig + 1))) {

      MOTORCAPUCHA.set(0);
    } else if (anguloFinal <= (-AnguloCapuchaConfig - 1)) {

      MOTORCAPUCHA.set(-0.25);

    }

  }

  public void PIDchasis() {

    double rampchasis = 0.3;

    MOTORD1ENC.configFactoryDefault();
    MOTORD2.configFactoryDefault();
    MOTORD3.configFactoryDefault();
    MOTORI4ENC.configFactoryDefault();
    MOTORI5.configFactoryDefault();
    MOTORI6.configFactoryDefault();

    MOTORD1ENC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    MOTORI4ENC.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    MOTORD1ENC.configOpenloopRamp(rampchasis);
    MOTORD2.configOpenloopRamp(rampchasis);
    MOTORD3.configOpenloopRamp(rampchasis);
    MOTORI4ENC.configOpenloopRamp(rampchasis);
    MOTORI5.configOpenloopRamp(rampchasis);
    MOTORI6.configOpenloopRamp(rampchasis);

  }

  public void autonomo_2_cargos_linea_fender() {

    AnguloCapuchaConfig = 18;
    ajusteDeTiroautonomo();
    velocidadesShooter.velocidad = 5050;

    switch (estadoAuto) {

      case 0:
        PISTINTAKE.set(true);
        MOTORINTAKE.set(0.6);

        estadoAuto = 1;
        break;

      case 1:
        if (Math.abs(distmeters) <= 1.35) {
          chasis.arcadeDrive(0, 0.54);
        } else {
          chasis.arcadeDrive(0, 0);

          estadoAuto = 2;
        }
        break;

      case 2:
        PISTINTAKE.set(false);
        estadoAuto = 3;
        break;

      case 3:
        if (Math.abs(navx.getAngle()) <= 150) {
          chasis.arcadeDrive(-0.64, 0);
        } else {
          chasis.arcadeDrive(0, 0);
          MOTORD1ENC.setSelectedSensorPosition(0);
          MOTORI4ENC.setSelectedSensorPosition(0);
          estadoAuto = 4;
        }
        break;

      case 4:
        if (Math.abs(distmeters) <= 0.33) {
          chasis.arcadeDrive(0, 0.5);
        } else {
          chasis.arcadeDrive(0, 0);
          estadoAuto = 5;
        }
        break;

      case 5:
        AutoInit = Timer.getFPGATimestamp();
        estadoAuto = 6;
        break;

      case 6:
        deltaMatchTime = time - AutoInit;
        ShooterPID(velocidadesShooter.tarmac);
        if (deltaMatchTime <= 4.75) {
          ajustedegiroautonomo();
        } else if (deltaMatchTime > 4.75 && deltaMatchTime <= 7.75) {
          MOTORINDEXER.set(0.23);
          MOTORINTAKE.set(0);
        } else if (deltaMatchTime > 7.5) {
          estadoAuto = 7;
        }
        break;

      case 7:
        MOTORD1ENC.setSelectedSensorPosition(0);
        MOTORI4ENC.setSelectedSensorPosition(0);
        estadoAuto = 8;
        break;

      case 8:
        AutonomoTaxi();
        break;

    }
  }

  public void autonomo_1_taxi_pegado_fender() {
    ajusteDeTiroautonomo();

    if (Timer.getMatchTime() <= 15 && Timer.getMatchTime() >= 10) {
      ShooterPID(-4800);
      AnguloCapuchaConfig = 6.8;

    } else {
      MOTORSHOOTERLEFT.set(0);
      MOTORSHOOTERRIGHT.set(-0);
    }

    if (Timer.getMatchTime() <= 12 && Timer.getMatchTime() >= 9.8) {
      MOTORINDEXER.set(0.4);
    } else {
      MOTORINDEXER.set(0);
      MOTORINTAKE.set(0);

    }

    if (Timer.getMatchTime() <= 9) {
      AutonomoTaxiPegadoFender();
    }

  }

  public void autonomo_taxi_a_inicios() {

    if (Timer.getMatchTime() < 14 && Timer.getMatchTime() > 12) {
      AutonomoTaxiPegadoFender();

    } else {

      chasis.arcadeDrive(0, 0);
    }
  }

  public void autonomo_taxi_a_finales() {

    if (Timer.getMatchTime() < 6 && Timer.getMatchTime() > 1) {
      AutonomoTaxiPegadoFender();

    } else {

      chasis.arcadeDrive(0, 0);
    }

  }

  public void lanzamiento_de_distintos_lados() {
    // calculos
    // distancia
    double y = ty.getDouble(0.0);
    double angleToGoalDegrees = LimeLight.anguloInclinacionLL + y;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanciaFender = (LimeLight.alturaUpperPulgadas - LimeLight.alturaAlPisoPugadasLL)
        / Math.tan(angleToGoalRadians);

    if (distanciaFender > 0 && distanciaFender < 90) {

      ajustedelanzamiento(60);
      AnguloCapuchaConfig = 18;
      velocidadesShooter.velocidad = -5000;
    }
    if (distanciaFender > 90 && distanciaFender < 140) {

      ajustedelanzamiento(115);
      AnguloCapuchaConfig = 19;
      velocidadesShooter.velocidad = -5450;
    }
    if (distanciaFender > 140 && distanciaFender < 260) {

      ajustedelanzamiento(160);
      AnguloCapuchaConfig = 23;
      velocidadesShooter.velocidad = -6050;
    }



  }

  public void AjustarAngulo() {
    if (anguloFinal >= (-anguloo + 1)) {
      MOTORCAPUCHA.set(0.5);
    } else if ((anguloFinal >= (-anguloo - 1)) && (anguloFinal <= (-anguloo + 1))) {
      MOTORCAPUCHA.set(0);
    } else if (anguloFinal <= (-anguloo - 1)) {
      MOTORCAPUCHA.set(-0.5);

    }

  }

  public void ajustarvelocidad() {
    if (JoystickDriver2.getRawButton(Kxbox.BOTONES.LB) && limitindexer.get() == true) {
      MOTORINDEXER.set(0.3);
    } else if (JoystickDriver2.getRawAxis(Kxbox.AXES.RT) >= 0.3) {
      MOTORINDEXER.set(0.3);
    } else {
      MOTORINDEXER.set(0);
    }


    // Apuntar
    if (JoystickDriver2.getRawAxis(Kxbox.AXES.LT) >= 0.3) {
      ShooterPID(velocidaad);
    } else {
      MOTORSHOOTERLEFT.set(0);
      MOTORSHOOTERRIGHT.set(-0);
    }

  }

}
