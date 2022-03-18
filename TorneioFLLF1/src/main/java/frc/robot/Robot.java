package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Motores
  VictorSPX m_right = new VictorSPX(1);
  VictorSPX m_right2 = new VictorSPX(2);
  VictorSPX m_left = new VictorSPX(3);
  VictorSPX m_left2 = new VictorSPX(4);
  VictorSPX m_armazem = new VictorSPX(7);
  VictorSPX m_lancamento = new VictorSPX(6);
  
  //Joystick
  Joystick m_stick = new Joystick(0);
  Joystick m_stick2 = new Joystick(1);

  Timer mTimer = new Timer();

  int tempoRotina = 30;
  int pov;
  int i;
  int pa;
  double limmin=0.2, medio=0.4, limmax=1, bt=limmin;
  double x1, y1, x2, y2, lt, rt;
  double mag, sin1, mag2, sin2;
  double pl, pr, pLanc;
  double t1, t2;
  boolean a, b, x, y, davi, daviA, daviX, daviB, gravador;
  double[][] vetpower = new double[2][(int)(tempoRotina/0.05)];

  public Robot(){
    super(0.05);
  }
  @Override
  public void robotInit(){
    
    //Motores Servos
    m_right2.follow(m_right);
    m_left2.follow(m_left);

    //Configura Modo Neutro para 4%
    m_left.configNeutralDeadband(0.04);
    m_left2.configNeutralDeadband(0.04);
    m_right.configNeutralDeadband(0.04);
    m_right2.configNeutralDeadband(0.04);

    //Travar motores caso força seja 0
    m_left.setNeutralMode(NeutralMode.Brake);
    m_left2.setNeutralMode(NeutralMode.Brake);
    m_right.setNeutralMode(NeutralMode.Brake);
    m_right2.setNeutralMode(NeutralMode.Brake);
    m_armazem.setNeutralMode(NeutralMode.Brake);
    m_lancamento.setNeutralMode(NeutralMode.Brake);
    
  }
  @Override
  public void teleopInit() {
    java.util.Arrays.fill(vetpower, 0);
    gravador=false;
    i=0;
    mTimer.reset();
    mTimer.start();
  }
 @Override
 public void teleopPeriodic(){

  // Limite da força
  a=m_stick.getRawButton(1);
  b=m_stick.getRawButton(2);
  x=m_stick.getRawButton(3);

  if(b){
    SmartDashboard.putBoolean("Minimo", true);
    SmartDashboard.putBoolean("Medio", false);
    SmartDashboard.putBoolean("Maximo", false);
    bt=limmin;
  }else if(a){
    SmartDashboard.putBoolean("Minimo", false);
    SmartDashboard.putBoolean("Medio", true);
    SmartDashboard.putBoolean("Maximo", false);
    bt=limmax;
  }else if(x){
    SmartDashboard.putBoolean("Minimo", false);
    SmartDashboard.putBoolean("Medio", false);
    SmartDashboard.putBoolean("Maximo", true);
    bt=limmax;
  }

  // atribuido valores a variaveis

  rt=m_stick.getRawAxis(3);
  lt=-m_stick.getRawAxis(2);
  x1=m_stick.getRawAxis(0);
  y1=-m_stick.getRawAxis(1);
  x2=m_stick.getRawAxis(4);
  y2=-m_stick.getRawAxis(5);
  pov=m_stick.getPOV();
  t1=(t1>=0.5)?0.5:(mTimer.get()-t1)*2;
  t2=(t2>=0.5)?0.5:(mTimer.get()-t2)*2;
  mag = Math.sqrt(Math.pow(x1, 2) + Math.pow(y1, 2));
  mag2 = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2));
  sin1=y1/mag;
  sin2=y2/mag2;
  mag = mag*t1*2*bt;
  mag2 = mag2*t2*2*bt;

  // zerando as variaveis em caso de neutro

  if(mag<0.1 && rt==0 && lt==0){
    t1=mTimer.get();
    mag=0;
    x1=0;
    y1=0;
  }
  if(mag2<0.1 && rt==0 && lt==0){
    t2=mTimer.get();
    mag2=0;
    x2=0;
    y2=0;
  }

  // calculo das forças

  if(rt!=0){
    if(x1>=0){
      if(x1<=0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","rt!=0 && x1>=0");
      pr=(1-x1)*rt;
      pl=rt;
    }else{
      if(x1>-0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","rt!=0 && x1<0");
      pr=rt;
      pl=(1+x1)*rt;
    }
  }
  else if(lt!=0){
    if(x1>=0){
      if(x1<0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","lt!=0 && x1>=0");
      pr=lt;
      pl=(1-x1)*lt;
    }else{
      if(x1>-0.04){
        x1=0;
     }
      SmartDashboard.putString("Condicao","lt!=0 && x1<0");
      pr=(1+x1)*lt;
      pl=lt;
    }   
  }
  else if(mag!=0){
    if(x1>=0 && y1>=0){
      SmartDashboard.putString("Condicao","x1>=0 && y1>=0");
      pr=sin1*mag;
      pl=mag;
    }
    else if(x1<0 && y1>=0){
      SmartDashboard.putString("Condicao","x1<0 && y1>=0");
      pr=mag;
      pl=sin1*mag;
    }
    else if(x1<0 && y1<0){
      SmartDashboard.putString("Condicao","x1<0 && y1<0");
      pr=sin1*mag;
      pl=-mag;
    }
    else if(x1>=0 && y1<0){
      SmartDashboard.putString("Condicao","x1>=0 && y1<0");
      pr=-mag;
      pl=sin1*mag;
    }
  }
  else if(mag2!=0){
    if(x2>=0 && y2>=0){
      SmartDashboard.putString("Condicao","x2>=0 && y2>=0");
      pr=-mag2;
      pl=-sin2*mag2;
    }
    else if(x2<0 && y2>=0){
      SmartDashboard.putString("Condicao","x2<0 && y2>=0");
      pr=-sin2*mag2;
      pl=-mag2;
    }
    else if(x2<0 && y2<0){
      SmartDashboard.putString("Condicao","x2<0 && y2<0");
      pr=mag2;
      pl=-sin2*mag2;
    }
    else if(x2>=0 && y2<0){
      SmartDashboard.putString("Condicao","x2<0 && y2<0");
      pr=-sin2*mag2;
      pl=mag2;
    }
  }
  else if(pov!=-1){
    if(pov==0){
      SmartDashboard.putString("Condicao","pov==0");
      pr=bt;
      pl=bt;
    }
    else if(pov==45){
      SmartDashboard.putString("Condicao","pov==45");
      pr=0;
      pl=bt;
    }
    else if(pov==90){
      SmartDashboard.putString("Condicao","pov==90");
      pr=-bt;
      pl=bt;
    }
    else if(pov==135){
      SmartDashboard.putString("Condicao","pov==135");
      pr=0;
      pl=-bt;
    }
    else if(pov==180){
      SmartDashboard.putString("Condicao","pov==180");
      pr=-bt;
      pl=-bt;
    }
    else if(pov==225){
      SmartDashboard.putString("Condicao","pov==225");
      pr=-bt;
      pl=0;
    }
    else if(pov==270){
      SmartDashboard.putString("Condicao","pov==270");
      pr=bt;
      pl=-bt;
    }
    else if(pov==315){
      SmartDashboard.putString("Condicao","pov==315");
      pr=bt;
      pl=0;
    }
  }else{
    SmartDashboard.putString("Condicao","else");
    pr=0;
    pl=0;
  }

  // execução dos motores da locomoção

  SmartDashboard.putNumber("Power Right", pr);
  SmartDashboard.putNumber("Power Left", pl);
  m_left.set(ControlMode.PercentOutput, pl);
  m_right.set(ControlMode.PercentOutput, -pr);
  

  //gravação

  y=m_stick.getRawButtonPressed(4);

  if(y){
    gravador=!gravador;
    i=0;
  }
  if(gravador){
      SmartDashboard.putBoolean("Gravador:", true);
      vetpower[0][i]=pl;
      vetpower[1][i]=pr;
      i++;
  }else{
    SmartDashboard.putBoolean("Gravador:", false);
  }
  SmartDashboard.putNumberArray("motor esquerdo", vetpower[0]);
  SmartDashboard.putNumberArray("motor direito", vetpower[1]);

  //Lançamento

  daviA = m_stick2.getRawButton(1);
  daviB = m_stick2.getRawButton(2);
  daviX = m_stick2.getRawButton(3);

  pLanc = (daviA)?1:(daviX)?0.75:(daviB)?0.5:0;
  pa= (daviA || daviB || daviX)?1:0;

  m_armazem.set(ControlMode.PercentOutput, pa);
  m_lancamento.set(ControlMode.PercentOutput, -pLanc);
  SmartDashboard.putNumber("ForcaLancamento", pLanc);
 }
 @Override
 public void autonomousInit() {
   i=0;
 }
 @Override
 public void autonomousPeriodic() {
  
  m_left.set(ControlMode.PercentOutput, vetpower[0][i]);
  m_right.set(ControlMode.PercentOutput, -vetpower[1][i]);
  SmartDashboard.putNumber("motor esquerdo", vetpower[0][i]);
  SmartDashboard.putNumber("motor direito", vetpower[1][i]);
  i++;
 }

}