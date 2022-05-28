package frc;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class P_Joystick{

    private static Joystick Stick = new Joystick(0);

    private static boolean a, b, x;
    private static final double[] LIMITES={1 , 0.4 , 0.2};
    private static double bt = LIMITES[2];
    private static boolean[] b_Limites = new boolean[3];

    public static double[] p = new double[4];

    private byte eixos[] =  new byte[2];
    Timer T_Timer = new Timer();
    
    public P_Joystick(String Lado){

        reset();
        eixos[0] = (byte)((Lado == "Esquerdo")?0:(Lado == "Direito")?4: (Lado == "G_Esquerdo")?2:1);
        eixos[1] = (byte)((Lado == "Esquerdo")?1:5);
    }

    public static double setBt(){

        a=Stick.getRawButton(1);
        b=Stick.getRawButton(2);
        x=Stick.getRawButton(3);
        b_Limites[2] = b?true:(a || x)?false:b_Limites[2];
        b_Limites[1] = a?true:(b || x)?false:b_Limites[1];
        b_Limites[0] = x?true:(a || b)?false:b_Limites[0];
        bt = b?LIMITES[2]:a?LIMITES[1]:x?LIMITES[0]:bt;
        
        SmartDashboard.putBoolean("Mínimo", b_Limites[2]);
        SmartDashboard.putBoolean("Médio", b_Limites[1]);
        SmartDashboard.putBoolean("Máximo", b_Limites[0]);

        return bt;
    }

    private void t_Reset(){
        T_Timer.reset();
        T_Timer.start();
    }

    public void reset(){

        b_Limites[0]=false;
        b_Limites[1]=false;
        b_Limites[2]=true;
        bt=LIMITES[2];  
        t_Reset();
    }

    private double t_getTimer(){

        return (T_Timer.get()>=0.5)?1:T_Timer.get()*2;
    }

    public double getEixo(byte i){
        
        return (Math.abs(Stick.getRawAxis(eixos[i]))<0.1)?0:Stick.getRawAxis(eixos[i]);
    }

    public double j_SetPower(){

        return Math.hypot(getEixo((byte) 0), getEixo((byte) 1))*t_getTimer()*setBt();
    }

    private double j_SetSen(){

        return getEixo((byte)0)/Math.hypot(getEixo((byte)0), getEixo((byte)1));
    }

    public double j_GetPowerTrigger(){

        return getEixo((byte)0)*setBt()*t_getTimer();
    }

    public void t_ResetNeutral(){

        if(j_SetPower() <0.1) t_Reset();
      
    }

    public double p_PercentualForca(double x, double limMinX, double limMaxX, double limMinY, double limMaxY){

        double a = (limMaxY-limMinY)/(limMaxX-limMinX);
      
        return a*(x - limMinX)+limMinY;
    }

    public void p_setPowerTrigger(){
        byte e = (byte)((eixos[0]==3)?1:-1);
        double a_X = Stick.getRawAxis(0);
        if(j_GetPowerTrigger()!=0){
          p[(a_X>=0)?0:1]=e*j_GetPowerTrigger()*p_PercentualForca(a_X, 0, (a_X>=0)?1:-1, 1, 0);
          p[(a_X>=0)?1:0]=e*j_GetPowerTrigger();
        }
     }

     public void p_setPowerStick(){
        byte f = (byte)((eixos[0]==0)?1:-1);
        p[(getEixo((byte)0)/getEixo((byte)1)>=0)?0:1]=f*((getEixo((byte)1)>=0)?1:-1)*j_SetPower()*p_PercentualForca(j_SetSen(), 0, (getEixo((byte)1)>=0)?1:-1, 0, 1);
        p[(getEixo((byte)0)/getEixo((byte)1)>=0)?1:0]=f*((getEixo((byte)1)>=0)?1:-1)*j_SetPower();
    }

    public static short pov(){

        return (short)Stick.getPOV();
    }
}
