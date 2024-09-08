int Sensor1 = A0;
int Sensor2 = A1;
int Sensor3 = A2;
int Sensor4 = A3;
int Sensor5 = A4;
int clp = 8;

int x=0;
// values from ir sensor
int S1;
int S2;
int S3;
int S4;
int S5;

// declaring motor pins
int enR = 5;   // right motor speed
int inl1 = 2;   // right motor direction
int inl2 = 4;   // right motor direction
int inr3 = 6;  // left motor direction
int inr4 = 7;  // left motor direction
int enL = 3; 
int threshold=100;
//light at end point
int ledpin = 9;

// motor speed with pid
int awpl;
int dwpl;
int awpr;
int dwpr;

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int drylfspeed = 70;
int actuallfspeed =190;


float Kp = 0;
float Kd = 0;
float Ki = 0;
int proportional=0;
int integral=0;
int derivative=0;
int last_proportional=0;
int right_speed=0;
int left_speed=0;
int sensors_sum=0;
int sensors_average=0;
int sensors[5]={0,0,0,0,0};
int Position=0;
int error_value=0;

char path[100]={};
char direction;
int pathlength=0;
int pathindex=0;
int lastError;
int shortDone=0;

void intersection(char direction){

  path[pathlength]=direction;
  pathlength++;

  shortpath();

}

void shortpath(){

  if(pathlength>=3 && path[pathlength-2]=='B')
  {
    shortDone=0;
    if(path[pathlength-3] == 'L' && path[pathlength - 1] == 'R')
  {
    pathlength -= 3;
    path[pathlength] = 'B';
    shortDone=1;
  }
  if(path[pathlength-3] == 'L' && path[pathlength - 1] == 'S' && shortDone == 0)
  {
    pathlength -= 3;
    path[pathlength] = 'R';
    shortDone = 1;
  }
  if(path[pathlength-3] == 'R' && path[pathlength - 1] == 'L' && shortDone == 0)
  {
    pathlength-=3;
    path[pathlength] = 'B';
    shortDone=1;
  }
  if(path[pathlength-3] == 'S' && path[pathlength - 1] == 'L' && shortDone == 0)
  {
    pathlength -= 3;
    path[pathlength] = 'R';
    shortDone = 1;
  }
  if(path[pathlength-3] == 'S' && path[pathlength - 1] =='S' && shortDone == 0)
  {
    pathlength-=3;
    path[pathlength] = 'B';
    shortDone=1;
  }
  if(path[pathlength-3] == 'L' && path[pathlength - 1] =='L' && shortDone == 0)
  {
    pathlength -= 3;
    path[pathlength] = 'S';
    shortDone = 1;
  }

}

  else if((path[pathlength-2]=='L' && path[pathlength-3]=='R') || (path[pathlength-2]=='R'&& path[pathlength-3]=='L') && shortDone ==0 ){
    pathlength-=3;
    shortDone=1;
  }
  


  // if(pathlength!<3 || (path[pathlength-2]='L' && path[pathlength-3] == 'L' && path[pathlength - 1] == 'L' && shortdone == 0)){
  //  pathlength = pathlength - 3;
  //  shortdone =1;
  
  // }
  
  // if((pathlength!<3) || (path[pathlength-2]='R'&& path[pathlength-3] =='R' && path[pathlength - 1] == 'R'&& shortdone == 0)){
  //  pathlength = pathlength - 3;
  //  shortdone = 1;
  // }
}

void finalpath(){

  switch(path[pathindex]){
   
   case 'S': slowmoveforward(); break;
   case 'L': turnleft(); break;
   case 'R': turnright(); break;
   case 'B': turnleft(); break;
   case 'E': digitalWrite(inl1,LOW);
    digitalWrite(inl2,LOW);
    digitalWrite(inr3,LOW);
    digitalWrite(inr4,LOW);
    analogWrite (enL, 0);
    analogWrite (enR, 0);
    break;
   
  }
}

void linefollow() 
  {
    int error = (analogRead(Sensor2) - analogRead(Sensor4));

    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    dwpl = drylfspeed - PIDvalue;
    dwpr = drylfspeed + PIDvalue;

    awpl = actuallfspeed - PIDvalue;
    awpr = actuallfspeed + PIDvalue;

    if (dwpl > 240) 
    {
      dwpl = 240;
    }
    if (dwpl < 0) 
    {
      dwpl = 0;
    }
    if (dwpr > 240) 
    {
      dwpr = 240;
    }
    if (dwpr < 0)
    {
      dwpr = 0;
    }

    if (awpl > 240) 
    {
      awpl = 240;
    }
    if (awpl < 0) 
    {
      awpl = 0;
    }
    if (awpr > 240) 
    {
      awpr = 240;
    }
    if (awpr < 0) 
    {
      awpr = 0;
    }
  }

void slowmoveforward() 
{ linefollow();
  analogWrite(enL, dwpl);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);

  analogWrite(enR, dwpr);
  digitalWrite(inr3, HIGH);
  digitalWrite(inr4, LOW);
}


void turnleft() 
{
  analogWrite(enL, dwpr);
  digitalWrite(inl1, LOW);
  digitalWrite(inl2, HIGH);
  analogWrite(enR, dwpl);
  digitalWrite(inr3, HIGH);
  digitalWrite(inr4, LOW);
}

void turnright() 
{
  analogWrite(enL, dwpl);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);
  analogWrite(enR, dwpr);
  digitalWrite(inr3, LOW);
  digitalWrite(inr4, HIGH);
}



void stop() 
{
  analogWrite(enL, 0);
  digitalWrite(inl1, LOW);
  digitalWrite(inl2, LOW);
  analogWrite(enR, 0);
  digitalWrite(inr3, LOW);
  digitalWrite(inr4, LOW);
}

void finish()
{
  analogWrite(enL, 0);
  digitalWrite(inl1, LOW);
  digitalWrite(inl2, LOW);
  analogWrite(enR, 0);
  digitalWrite(inr3, LOW);
  digitalWrite(inr4, LOW);
  digitalWrite(ledpin, HIGH);
  delay(50000);
  digitalWrite(ledpin, LOW);
    x=2;
}
void calibrationspeed()
{
  analogWrite(enL,70);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);
  analogWrite(enR, 70);
  digitalWrite(inr3, LOW);
  digitalWrite(inr4, HIGH);
}

// sensor readings

void readsensor() 
{
  //if declared outside, the sensor readings become “11111”
  S1 = analogRead(A0);
  S2 = analogRead(A1);
  S3 = analogRead(A2);
  S4 = analogRead(A3);
  S5 = analogRead(A4);
}


void conditions()
   {

    if(S1>threshold && S2>threshold && S3>threshold && S4>threshold && S5>threshold)
    {
      turnleft();
      delay(500);
      stop();
     
      delay(2000);
       readsensor();
      if(S1>threshold && S2>threshold && S3>threshold && S4>threshold && S5>threshold)
      {
        finish();
          intersection('E');
          path[pathlength]='E';
          pathlength=pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;
          }
      }
      else
      {
        turnleft();
          delay(500);
          intersection('L');
          path[pathlength]='L';
          pathlength=pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;

      
          }    
     }
    }

    else if(S1>threshold && S2>threshold)
    {
      turnleft(); 
          intersection('L');
          path[pathlength]='L';
          pathlength=pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;
          }
    }

    else if(S3<threshold && S4>threshold && S5>threshold )
    {
      turnright();
        intersection('R');
        path[pathlength]='R';
       pathlength = pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;
          }
    }

    else if(S1<threshold && S2<threshold && S3>threshold && S4<threshold && S5<threshold )
    {

     slowmoveforward();
        intersection('S');
       path[pathlength]='S';
       pathlength = pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;
          }
     
    }
    else if (S1<threshold && S2<threshold && S3<threshold && S4<threshold && S5<threshold)
    {
      turnleft();
             intersection('B');
        path[pathlength]='B';
        pathlength=pathlength + 1;
          if(path[pathlength - 1] == path[pathlength])
          {
            pathlength = pathlength -1;
          }
    }

  }

void calibrate()
{
   int minValue = S3;
    int maxValue = S3;
    for(int i=0; i<80 ; i++){
  calibrationspeed();
  delay(100);  
    readsensor();                       
      if (S3 < minValue)
      {
        minValue = S3;
      }
      if (S3 > maxValue)
      {
        maxValue=S3;
      }
    
    }
      x=1;
    threshold = (minValue + maxValue) / 2;
  stop();
}

  void setup() 
  {
    Serial.begin(9600);

    // setting pins
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);

    pinMode(inl1, OUTPUT);
    pinMode(inl2, OUTPUT);
    pinMode(inr3, OUTPUT);
    pinMode(inr4, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(enR, OUTPUT);
    pinMode(clp, INPUT_PULLUP);
    pinMode(ledpin, OUTPUT);
  }


void loop() {
  // put your main code here, to run repeatedly
  while(x==0)
  {
    calibrate();
    digitalWrite(ledpin,HIGH);
    delay(10000);
    digitalWrite(ledpin,LOW);
  }

  delay(1000);
while(x==1)
{
readsensor();
conditions();
}

while(x== 2){
   if (S1>threshold && S2>threshold && S3>threshold && S4>threshold && S5>threshold )
   {
     finalpath();
   }
   else if(S1>threshold && S2>threshold)
    {
     finalpath();
    }
   else if(S3<threshold && S4>threshold && S5>threshold )
    {
     finalpath();
    }
    else{
     conditions();
    }

}


}
