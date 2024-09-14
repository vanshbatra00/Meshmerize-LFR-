int Sensor1 = A0;
int Sensor2 = A1;
int Sensor3 = A2;
int Sensor4 = A3;
int Sensor5 = A4;


int x=0;
// values from ir sensor
int S1;
int S2;
int S3;
int S4;
int S5;

// declaring motor pins
int enR = 6;   // right motor speed
int inl1 = 2;   // right motor direction
int inl2 = 4;   // right motor direction
int inr3 = 5;  // left motor direction
int inr4 = 7;  // left motor direction
int enL = 3; 
int threshold=0;


// motor speed with pid
int awpl=0;
int dwpl=0;
int awpr=0;
int dwpr=0;

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int drylfspeed = 70;
int actuallfspeed =70;

float Kp = 0;
float Kd = 0;
float Ki = 0;



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
{ 
  // linefollow();
  analogWrite(enL, 80);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);

  analogWrite(enR, 100);
  digitalWrite(inr3, HIGH);
  digitalWrite(inr4, LOW);
}


void turnleft() 
{ 
  // linefollow();
  analogWrite(enL, 70);
  digitalWrite(inl1, LOW);
  digitalWrite(inl2, HIGH);
  analogWrite(enR, 100);
  digitalWrite(inr3, HIGH);
  digitalWrite(inr4, LOW);
}

void turnright() 
{ 
  // linefollow();
  analogWrite(enL, 70);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);
  analogWrite(enR, 100);
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

}
void calibrationspeed()
{
  analogWrite(enL,100);
  digitalWrite(inl1, HIGH);
  digitalWrite(inl2, LOW);
  analogWrite(enR, 140);
  digitalWrite(inr3, LOW);
  digitalWrite(inr4, HIGH);
}

// sensor readings

void readsensor() 
{
  S1 = analogRead(Sensor1);
  S2 = analogRead(Sensor2);
  S3 = analogRead(Sensor3);
  S4 = analogRead(Sensor4);
  S5 = analogRead(Sensor5);
}


void conditions()
{  
    if(S1>threshold && S2>threshold && S3>threshold && S4>threshold && S5>threshold)
    {
      turnleft();
      delay(100);
      stop();
      readsensor();

      if(S1>threshold && S2>threshold && S3>threshold && S4>threshold && S5>threshold)
      {
        finish();
      }
      else
      {
        turnleft();  
      }
    }

    else if(S1>threshold && S2>threshold)
    {
      turnleft(); 
    }

    else if(S3>threshold && S4>threshold && S5>threshold )
    {
      slowmoveforward();
    }

    else if(S1<threshold && S2<threshold && S3>threshold && S4<threshold && S5<threshold )
    {

     slowmoveforward();
    }
    
    else if(S3<threshold && S4>threshold && S5>threshold )
    {
      turnright();
      delay(20);
    }

    else if (S1<threshold && S2<threshold && S3<threshold && S4<threshold && S5<threshold)
    {
      turnleft();
    }

}

void calibrate()
{
    int minValue = S3;
    int maxValue = S3;
    for(int i=0; i<80 ; i++)
    {
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

  }


void loop()
{
  while(x==0)
  {
    calibrate();
    x=1;
  }
  delay(1000);

  while(x==1)
  {
    readsensor();
    conditions();
  }
}