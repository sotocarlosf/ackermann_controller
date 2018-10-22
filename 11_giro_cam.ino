#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <Servo.h>           //Include The servo library for the functions used
#include <TimerOne.h>

//--------------------------------- Valores de la vida real ------------------------------------
float diametro_rueda = 34.0;
float periodo_muestreo_milis = 100;
float tics_vuelta_D = 79.0;
float tics_vuelta_I = 78.0;


//------------------------------------- Interrupciones -------------------------------------------
long int numDER, numIZQ;                //CUENTA TOTAL DE CUADROS
long int num0DER, num1DER, num0IZQ, num1IZQ;
boolean tder=false;
boolean tizq=false;

//---------------------------------move_velocity() & move_direction() ------------------------
float usCurva = 1600;     //uS A ESCRIBIR EN SERVO Y ESC
float us = 1500;
Servo servo;                    //DECLARACION SERVO Y ESC
Servo esc;

//------------------------------------- callback() -------------------------------------------
long int cd,ci,oldDer,oldIzq;   //CUENTA DE CUADROS POR CICLO
float velD,velI;                //VELOCIDAD CM/S
int vueltas_D=0, vueltas_I=0;
float vdp=0.0,vd1=0.0,vd2=0.0,vd3=0.0,vd4=0.0,vd5=0.0,vip=0.0,vi1=0.0,vi2=0.0,vi3=0.0,vi4=0.0,vi5=0.0;
float Vel_actual=0.0;
float Curva_actual=0.0;
float grados_deseados = 0.0;

//-------------------------------------- pid_vel() --------------------------------------------
float Kvel=0.1;
float KvelI=0.0;
float errorVel, errorVelI, Vel_deseada, vel, vel_1;
//-------------------------------------- pid_dir() --------------------------------------------
float kp_dir, errorDir, theta, radio_deseado, theta_1,wk,Curva_deseada;
float L = 25.0;  //DIferencia entre las ruedas derechas e izquierdas


//-------------------------------------  ROS  -------------------------------
ros::NodeHandle  nh;

void w_comandos(const geometry_msgs::Point& cmd_msg){ //ENTRADA EN CM/S
    Vel_deseada = cmd_msg.x;
    grados_deseados = cmd_msg.y;
    Kvel = cmd_msg.z;
    
    usCurva = 1600 + grados_deseados;
    usCurva = max(1080, min(usCurva, 1920));
    servo.writeMicroseconds(int(usCurva));
    
    callback_control();
}



//--------------------------------------- Subscribers y Publishers ----------------------
std_msgs::Float32 velI_msg;
std_msgs::Float32 velD_msg;

ros::Publisher pub_velI("velI", &velI_msg);
ros::Publisher pub_velD("velD", &velD_msg);

ros::Subscriber<geometry_msgs::Point> sub_comandos("comandos", w_comandos);



void setup()
{
  //Serial.begin(57600);        //ROS Serial speed
  pinMode(2, INPUT_PULLUP);   //Encoder 3 DER
  pinMode(3, INPUT_PULLUP);   //Encoder 4 DER
  pinMode(7, OUTPUT);         //PWM_SERVO
  pinMode(8, OUTPUT);         //PWM_ESC
  pinMode(18, INPUT_PULLUP);  //Encoder 1 IZQ
  pinMode(19, INPUT_PULLUP);  //Encoder 2 IZQ
  pinMode(20, INPUT_PULLUP);  //SDA
  pinMode(21, INPUT_PULLUP);  //SCL
  pinMode(24, INPUT_PULLUP);  //INT_MPU6050
  pinMode(26, OUTPUT);        //AD0_MPU6050
  digitalWrite(26, LOW);

   
  servo.attach(7);               //Attach the SERVO to Digital Pin 7
  esc.attach(8);                 //Attach the ESC to Digital Pin 8


  attachInterrupt(digitalPinToInterrupt(2), INT_0DER, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), INT_1DER, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), INT_0IZQ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), INT_1IZQ, CHANGE);
  
  //Timer1.initialize(periodo_muestreo_milis*1000);         // initialize timer1, and set a (X) second period. microsenconds
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  esc.writeMicroseconds(1500);
  servo.writeMicroseconds(1600);
  delay(2000); 
  
  nh.initNode();
  nh.advertise(pub_velD);
  nh.advertise(pub_velI);

  nh.subscribe(sub_comandos);

  vel_1=1500;
  theta_1=1600;
}


void loop()
{
  delay(100);
  pub_velD.publish(&velD_msg);
  pub_velI.publish(&velI_msg);
  nh.spinOnce();

}


void pid_vel(){
  /*-------------------------------------VELOCIDAD-----------------------*/
  errorVel = Vel_deseada - Vel_actual;
  us = vel_1 + errorVel*Kvel;
  
  us = max(1520, min(us, 1700));
  esc.writeMicroseconds(int(us));
  
  vel_1 = us;
  
  }

  
void pid_dir(){
  /*-------------------------------------DIRECCION-----------------------*/
  usCurva = 1600 + grados_deseados;
  usCurva = max(1080, min(usCurva, 1920));
  servo.writeMicroseconds(int(usCurva));
  }


//--------------------------------Métodos de interrupción para encoders-----------------------------
void INT_0DER() {
  //if (tder){
    numDER++;
  //}
  //tder=false;
}
void INT_1DER(){
  tder= true;
}
void INT_0IZQ() {
  //if(tizq){
    //numIZQ++;
  //}
  //tizq = false;
}
void INT_1IZQ(){
  //tizq=true;
  numIZQ++;
}

//--------------------------------Callback ejecutado cada T = 200ms ----------------------------------
void callback_control() // vel = (TICS CONTADOS * DIAMETRO ) / (TICS*VUELTA * T)
{
  if(oldDer<=numDER){
    cd=numDER-oldDer;
    vd1 = (diametro_rueda * cd) / (tics_vuelta_D*periodo_muestreo_milis/1000) ;
    velD=(vd1+vd2+vd3+vd4+vd5)/5;
    vd5=vd4;vd4=vd3;vd3=vd2;vd2=vd1;
    
  }
  if(oldIzq<=numIZQ){
    ci=numIZQ-oldIzq;
    vi1 = (diametro_rueda * ci) / (tics_vuelta_I*periodo_muestreo_milis/1000);
    velI = (vi1+vi2+vi3+vi4+vi5)/5;
    vi5=vi4;vi4=vi3;vi3=vi2;vi2=vi1;
  }
  oldDer=numDER;
  oldIzq=numIZQ;
  Vel_actual = (velD + velI)/2;
  
  pid_vel();
  //pid_dir();
  
  velD_msg.data = velD;
  velI_msg.data = velI;
  //velD_msg.data = Vel_actual;
  //velI_msg.data = us;
}
