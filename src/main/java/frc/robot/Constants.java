
package frc.robot;

//AQUI ESTAN TODAS LAS VARIABLES, ESTAS SE PUEDEN EDITAR MAS FACIL DESDE AQUI//

public class Constants {


    public static final class Motores { // CONSTANTES DE MOTORES DEL ROBOT

        public static final class Chasis {
            public static int KMOTORD1 = 1;// Chassis derecha 1
            public static int KMOTORD2 = 2;// Chassis derecha 2
            public static int KMOTORD3 = 3;// Chassis derecha 3
            public static int KMOTORI4 = 6;// Chassis izquierda 1
            public static int KMOTORI5 = 5;// Chassis izquierda 2
            public static int KMOTORI6 = 4;// Chassis izquierda 3
        }

        public static final class Shooter { // Motores
            public static int KMOTORSLeft = 7;// Shooter 1
            public static int KMOTORSRight = 8;// Shooter 2

        }

        public static final class Intake { // Motores
            public static int KMOTORINTAKE = 9;// Intake

        }

        public static final class Capucha { // Motores
            public static int KMOTORCAPUCHA = 10;// Capucha

        }

        public static final class Indexer { // Motores
            public static int KMOTORINDEXER = 11;// Indexer

        }

        public static final class Climber { // Motores

            public static int KMOTORCLIMBER = 20;
        }

    }

    public static final class Controles { // CONSTANTES DE LOS PUERTOS DE LOS CONTROLES

        public static int kJoystickDriver1 = 0;
        public static int KJoystickDriver2 = 1;

    }

    public static final class Neumatica { // // CONSTANTES DE LOS PUERTOS DE SOLENOIDES

        public static int KPISTINTAKE1 = 0; // Piston intake
        public static int KPISTINTAKE2 = 1; // Piston intake

        public static int KPISTCHASIS1 = 2; // Piston para cambios
        public static int KPISTCHASIS2 = 3; // Piston para cambios

    }

    public static final class statusrobot { // CONSTANTES TIPO BOOLEAN PARA BOTONES DE CONTROL

        public static boolean IntakeState = false;
        public static boolean compresorState = false;

        public static boolean velocidadsh1 = false;
        public static boolean velocidadsh2 = false;

        public static boolean shooter = false;
        public static boolean indexer = false;

    }

    public static final class VelocidadChasis { // AQUI SE CONTROLA LA VELOCIDAD MAXIMA DEL CHASIS

        public static double velocidadX = 0.9;
        public static double velocidadgiro = 0.85;

    }

    public static final class LimeLight {

        public static double kp = -0.04;

  // ¿Cuántos grados hacia atrás gira su centro de atención desde la posición
  // perfectamente vertical?
  public static double anguloInclinacionLL = 39.5;
  // distancia desde el centro de la lente Limelight hasta el suelo
  public static double alturaAlPisoPugadasLL = 22.4;
  // distancia del objetivo al suelo
  public static double alturaUpperPulgadas = 105.1 ; // distancia hub 103.9 in

    }

    public static final class LimitSwitches {

        public static int capucha = 0;
        public static int indexer=2;
        

    }

    public static final class Kxbox { 
        /*
        * CONSTANTES PARA PROGRAMAR MAS RAPIDO EL CONTROL (FUNCIONA PARA SABER EL
        * NUMERO DE ENTRADA DE CADA BOTON)
        */
        public static final class BOTONES {
            // JoystickDriver1.getRawButton();
            public static int A = 1;
            public static int B = 2;
            public static int X = 3;
            public static int Y = 4;
            public static int LB = 5;
            public static int RB = 6;
            public static int boton_con_cuadritos = 7;
            public static int boton_con_lineas = 8;
            public static int boton_joystick_left = 9;
            public static int boton_joystick_right = 10;
        }

        public static final class AXES {
            // JoystickDriver1.getRawAxis(axis);
            public static int joystick_izquierdo_eje_X = 0;
            public static int joystick_izquierdo_eje_Y = 1;
            public static int LT = 2;
            public static int RT = 3;
            public static int joystick_derecho_eje_X = 4;
            public static int joystick_derecho_eje_Y = 5;

        }

        public static final class POV {
            // JoystickDriver1.getRawAxis(axis);
            public static int arriba = 0;
            public static int arriba_derecha = 45;
            public static int derecha = 90;
            public static int derecha_abajo = 135;
            public static int abajo = 180;
            public static int abajo_izquierda = 225;
            public static int izquierda = 270;
            public static int izquierda_arriba = 315;

        }
    }

    public static final class KPIDShooter { // CONSTANTES PARA PID DEL SHOOTER (DE PREFERENCIA NO LE MUEVAS)
        public static double torpm = 0.0003333333; // 1/2500 //variable, 1/velocidad maxima
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static GainsShoot kGains_Velocit = new GainsShoot(0.003, 0.000, 0.005, 1023.0 / 20660.0, 300, 1.00);

        // public static double rpmtotal; //Variable para controlar las rpm

    }

    public static final class KPIDCapucha {
      	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
    public final static GainsCapucha kGains_Velocit = new GainsCapucha( 0.25, 0.001, 20, 1023.0/7200.0,  300,  1.00);

    }

    public static final class velocidadesShooter {

        public static final double fender = -2000;
        public static final double tarmac = -5150;
        public static final double launchpad = -6050;

        public static double velocidad;
    }

    public static final class anguloCapucha {

        public static final double fender = 25;
        public static final double tarmac = 20;
        public static final double  launchpad = 17;
        public static double velocidad;
    }

}