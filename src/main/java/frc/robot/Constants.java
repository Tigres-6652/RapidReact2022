
package frc.robot;

//AQUI ESTAN TODAS LAS VARIABLES, ESTAS SE PUEDEN EDITAR MAS FACIL DESDE AQUI//

public class Constants {

    public static final class ControlarMecanismos {

        public static int intake = Kxbox.BOTONES.A;
        public static int compresor = Kxbox.BOTONES.B;
        public static int shifter1 = Kxbox.POV.derecha;
        public static int shifter2 = Kxbox.POV.izquierda;

    }

    public static final class Motores { // CONSTANTES DE MOTORES DEL ROBOT

        public static final class Chasis {
            public static int KMOTORD1 = 1;// Chassis derecha 1
            public static int KMOTORD2 = 2;// Chassis derecha 2
            public static int KMOTORD3 = 3;// Chassis derecha 3
            public static int KMOTORI4 = 4;// Chassis izquierda 1
            public static int KMOTORI5 = 5;// Chassis izquierda 2
            public static int KMOTORI6 = 6;// Chassis izquierda 3
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

            public static int KMOTORCLIMBER = 12;
        }

    }

    public static final class Controles { // CONSTANTES DE LOS PUERTOS DE LOS CONTROLES

        public static int kJoystickDriver1 = 0;
        public static int KJoystickDriver2 = 1;

    }

    public static final class Neumatica { // // CONSTANTES DE LOS PUERTOS DE SOLENOIDES

        public static int KPISTINTAKE = 0; // Piston intake
        public static int KPISTCHASIS1 = 1; // Piston para cambios
        public static int KPISTCHASIS2 = 2; // Piston para cambios


    }

    public static final class statusrobot { // CONSTANTES TIPO BOOLEAN PARA BOTONES DE CONTROL

        public static boolean IntakeState = false;
        public static boolean compresorState = false;

        public static boolean velocidadsh1 = false;
        public static boolean velocidadsh2 = false;

    }

    public static final class VelocidadChasis { // AQUI SE CONTROLA LA VELOCIDAD MAXIMA DEL CHASIS

        public static double velocidadX = 0.8;
        public static double velocidadgiro = 0.7;

    }

    public static final class Kxbox { /*
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
            public static int LB = 2;
            public static int RB = 3;
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

    public static final class KPIDShooter {     //CONSTANTES PARA PID DEL SHOOTER (DE PREFERENCIA NO LE MUEVAS)
        public static double torpm = 0.0003333333; // 1/2500 //variable, 1/velocidad maxima
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static Gains kGains_Velocit = new Gains(0.1, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);

        // public static double rpmtotal; //Variable para controlar las rpm

    }
}