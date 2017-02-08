void setup() {
  // put your setup code here, to run once:
pinMode(A0,INPUT);
pinMode(22,OUTPUT);
digitalWrite(22,HIGH);
Serial.begin(9600);
}
int weight;
void loop() {
  // put your main code here, to run repeatedly:
  weight=analogRead(A0);
  Serial.println(weight);
}
