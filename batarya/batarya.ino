
float referans = 300.0;
const int giris=100;
float total =0;
float average=0;
float okunandeger[giris];
float deger =0;
float volt =0;
float yuzde =-1;
void setup() {
  Serial.begin(115200);//Haberleşme için
  
 
}
void loop() {

  for(int i=0; i<100;i++){
  deger = analogRead(34);
  
  volt = ((deger/1202)*345)/15;
  
  okunandeger[i]= volt;
  total= total + okunandeger[i];
  
   average= total/giris;
  }
  total=0;
  
  Serial.println("Okunan Analog Değer: ");
  Serial.println(deger);
  Serial.print ("Voltaj(mV): ");
  Serial.print (average);
  Serial.println (" V");
  Serial.println ("--------------");
 delay(2000);


  yuzde = map(average, 31.1, 42.1, 0.0, 100.0 );
  Serial.print("batarya:");
  Serial.println(yuzde);
  
}
