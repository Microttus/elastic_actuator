//Define pin numbers
int motorpin1 = 7;
int motorpin2 = 8;
int enablepin = 9;
int buttonpin1 = 6;
int buttonpin2 = 10;

int direc = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(enablepin, OUTPUT); 
  pinMode(buttonpin1, INPUT);
  pinMode(buttonpin2, INPUT);

  //Pullup
  analogWrite(buttonpin1, LOW);
  analogWrite(buttonpin2, LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:   
  int PWM = 150;
  //Controlling speed (0 = off and 255 = max speed):
  analogWrite(enablepin, PWM); //ENA pin

  // Read buttons states
  int pinValue1 = digitalRead(buttonpin1);
  //Serial.print(pinValue1);
  
  int pinValue2 = digitalRead(buttonpin2);
  //Serial.print(pinValue2);



  if (pinValue1 == LOW && pinValue2 == HIGH) {
    direc = 1;
    }
  else if (pinValue1 == HIGH && pinValue2 == LOW) {
    direc = 2;
    }

  if (direc == 2) {
    digitalWrite(motorpin1, HIGH);
    digitalWrite(motorpin2, LOW);
    }
  else if (direc == 1) {
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, HIGH);
    }


}
