
package frc.robot;

//AQUI ESTAN TODAS LAS VARIABLES, ESTAS SE PUEDEN EDITAR MAS FACIL DESDE AQUI//

public class Constants {

    public static final class Motores { // Motores

        public static int KMOTORD1 = 1;// Chassis derecha 1
        public static int KMOTORD2 = 2;// Chassis derecha 2
        public static int KMOTORD3 = 3;// Chassis derecha 3
        public static int KMOTORI4 = 4;// Chassis izquierda 1
        public static int KMOTORI5 = 5;// Chassis izquierda 2
        public static int KMOTORI6 = 6;// Chassis izquierda 3
        public static int KMOTORS1 = 7;// Shooter 1
        public static int KMOTORS2 = 8;// Shooter 2
        public static int KMOTORIN = 9;// Intake
        public static int KMOTORCP = 10;// Capucha
        public static int KMOTORID = 11;// Indexer

    }

    public static final class Controllers { // Motores

        public static int kJoystickDriver1 = 0;// Chassis derecha 1
        public static int KJoystickDriver2 = 1;// Chassis derecha 1

    }

    public static final class Neumatica { // Motores

        public static int KPISTINTAKE = 0; // Piston intake
        public static int KPISTCHASIS = 1; // Piston para cambios

    }

    public static final class statusrobot { // Motores

        public static boolean IntakeState;
        public static boolean compresorState;;

        

    }

    public static final class velocidades { // Motores

        public static double velocidad;
        public static double velocidadX=0.75;
        public static double velocidadgiro=0.5;

    }




}
