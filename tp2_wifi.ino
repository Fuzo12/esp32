/*
Sistemas Embebidos TP2-Estação Meterológica
Trabalho realizado por:
  Cláudio Coelho a22106474
  Lucas Martins a22103318
Trabalho realizado com base em:
  https://www.youtube.com/watch?v=K98h51XuqBE&list=PL9LvcY9iaJ_pvo435QoZAzvPvPQay8GZs&index=3
  https://www.youtube.com/watch?v=Va-psZoTBkY&list=PL9LvcY9iaJ_pvo435QoZAzvPvPQay8GZs&index=4
  https://www.youtube.com/watch?v=H62xzxI-4A0&list=PL9LvcY9iaJ_pvo435QoZAzvPvPQay8GZs&index=5
  https://www.youtube.com/watch?v=N38VBr4i0tI&list=PL9LvcY9iaJ_pvo435QoZAzvPvPQay8GZs&index=3&t=378s
  https://srituhobby.com/how-to-make-a-weather-monitoring-system-with-esp32-board/
  https://www.youtube.com/watch?v=W1xG_XJb0FU
  https://community.blynk.cc/t/problem-sending-sensor-data-to-blynk/63042/21
*/

#define BLYNK_TEMPLATE_ID "TMPL5t4Jd4966" //ID da template na plataforma Blynk.
#define BLYNK_TEMPLATE_NAME "Estação Meterológica" //Nome da template na plataforma Blynk.
//#define BLYNK_AUTH_TOKEN "J9_OIu2FGAEGW0LUSXXdYeLxWXNCFr81"

#include <DHT.h> //Biblioteca que permite a leitura de um sensor de humidade e temperatura DHT.
#include <Wire.h> //Biblioteca que permite a comunicação com dispositivos I2C/TWI.
#include <WiFi.h> //Biblioteca que permite a conexão a uma rede.
#include <LiquidCrystal_I2C.h> //Biblioteca que permite o controlo de um display LCD com adaptrador I2C (https://github.com/johnrickman/LiquidCrystal_I2C).
#include <BlynkSimpleEsp32.h> //Biblioteca que permite comunicação entre o Esp32 e a plataforma Blynk.

#define LDR 35
#define Rain 36
#define BTemp 0
#define BHum 15
#define BLight 32
#define BRain 4
DHT dht(5,DHT11); //Configura a biblioteca DHT.h, declarando o pino a que o sensor DHT está conectado e o seu tipo.
LiquidCrystal_I2C lcd(0x27,16,2);  //Configura a biblioteca LiquidCrystal_I2C.h referindo o endereço do LCD e o seu número de colunas e linhas.
BlynkTimer timer;

//Token de autentificação Blynk
char auth[] = "J9_OIu2FGAEGW0LUSXXdYeLxWXNCFr81";

//Rede de Casa.
/*
char ssid[] = "ZON-ECE0"; //Nome da Rede.
char pass[] = "11f5fe99716b"; //Password da Rede.
*/
//Rede ISMAT.
char ssid[] = "freeismat"; //Nome da Rede.
char pass[] = ""; //Password da Rede.

void setup() {
  pinMode(BTemp, INPUT_PULLUP);
  pinMode(BHum, INPUT_PULLUP);
  pinMode(BLight, INPUT_PULLUP);
  pinMode(BRain, INPUT_PULLUP);
  Serial.begin(115200); //Permite a realização de prints no Serial Monitor.
  dht.begin(); //Inicia a biblioteca DHT.h.
  lcd.init(); //Inicia a biblioteca LiquidCrystal_I2C.h.
  lcd.backlight(); //Ativa a luz de fundo no LCD.
  Blynk.begin(auth,ssid,pass); //Inicia a biblioteca BlynkSimpleEsp32.h, usando como parâmetros o Token de Autentificação fornecido pela plataforma, o nome da rede a que o dispositivo está conectado e a password do último.
}

int bTempState = 0; //Declara o estado atual do botão conectado ao pino D0.
int bHumState = 0; //Declara o estado atual do botão conectado ao pino D15.
int bLightState = 0; //Declara o estado atual do botão conectado ao pino D32.
int bRainState = 0; //Declara o estado atual do botão conectado ao pino D4.
bool watchTemperature = true; //Define se o valor da temperatura é exposto no LCD.
bool watchHumidity = false; //Define se o valor da humidade é exposto no LCD.
bool watchLDR = false; //Define se o valor da luminosidade é exposto no LCD.
bool watchRain = false; //Define se o valor da intensidade da chuva é exposto no LCD.

void loop() {
  Blynk.run(); //Permite o processamento de comandos e permite a manutenção da conexão com a plataforma Blynk.
  bTempState = digitalRead(BTemp); //Verifica o estado atual do botão conectado ao pino D0.
  if(!bTempState == HIGH){ //Verifica se o botão conectado ao pino D0 foi carregado.
    lcd.clear(); //Remove o que está a ser exposto no LCD.
    watchTemperature = true; //Permite que o valor da temperatura seja exposto no LCD.
    watchHumidity = false; //Impede que o valor da humidade seja exposto no LCD.
    watchLDR = false; //Impede que o valor da luminosidade seja exposto no LCD.
    watchRain = false; //Impede que o valor da intensidade da chuva seja exposto no LCD.
  }
  bHumState = digitalRead(BHum); //Verifica o estado atual do botão conectado ao pino D15.
  if(!bHumState == HIGH){ //Verifica se o botão conectado ao pino D15 foi carregado.
    lcd.clear(); //Remove o que está a ser exposto no LCD.
    watchTemperature = false; //Impede que o valor da temperatura seja exposto no LCD.
    watchHumidity = true; //Permite que o valor da humidade seja exposto no LCD.
    watchLDR = false; //Impede que o valor da luminosidade seja exposto no LCD.
    watchRain = false; //Impede que o valor da intensidade da chuva seja exposto no LCD.
  }
  bLightState = digitalRead(BLight); //Verifica o estado atual do botão conectado ao pino D32.
  if(!bLightState == HIGH){ //Verifica se o botão conectado ao pino D32 foi carregado.
    lcd.clear(); //Remove o que está a ser exposto no LCD.
    watchTemperature = false; //Impede que o valor da temperatura seja exposto no LCD.
    watchHumidity = false; //Impede que o valor da humidade seja exposto no LCD.
    watchLDR = true; //Permite que o valor da luminosidade seja exposto no LCD.
    watchRain = false; //Impede que o valor da intensidade da chuva seja exposto no LCD.
  }
  bRainState = digitalRead(BRain); //Verifica o estado atual do botão conectado ao pino D4.
  if(!bRainState == HIGH){ //Verifica se o botão conectado ao pino D4 foi carregado.
    lcd.clear(); //Remove o que está a ser exposto no LCD.
    watchTemperature = false; //Impede que o valor da temperatura seja exposto no LCD.
    watchHumidity = false; //Impede que o valor da humidade seja exposto no LCD.
    watchLDR = false; //Impede que o valor da luminosidade seja exposto no LCD.
    watchRain = true; //Permite que o valor da intensidade da chuva seja exposto no LCD.
  }
  readDHT(); //Função utilizada para obter leituras do sensor DHT11 e enviá-las para a plataforma Blynk.
  readLDR(); //Função utilizada para obter leituras do sensor LDR e enviá-las para a plataforma Blynk.
  readRain(); //Função utilizada para obter leituras do sensor de chuva e enviá-las para a plataforma Blynk.
  Serial.println("");
}

void readDHT(){
  float temperature = dht.readTemperature(); //Realiza a leitura da temperatura captada pelo sensor, aceita parâmetros mas não são inseridos para que a temperatura seja medida em Graus Celsius.
  float humidity = dht.readHumidity(); //Realiza a leitura da humidade captada pelo sensor.
  Blynk.virtualWrite(V0, temperature); //Envia o valor da temperatura lido pelo sensor para a plataforma Blynk, na qual a temperatura foi associada ao pino virtual 0.
  Blynk.virtualWrite(V1, humidity); //Envia o valor da humidade lido pelo sensor para a plataforma Blynk, na qual a humidade foi associada ao pino virtual 1.
  //Realiza print dos valores das leituras de temperatura e humidade do sensor no Serial Monitor.
  Serial.print("Temp: ");
  Serial.print(temperature);
  Serial.println("ºC");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  if(watchTemperature){ //Verifica se é permitido expor o valor da temperatura no LCD.
    lcd.setCursor(0,0); //Posiciona o cursor do LCD na primeira coluna da primeira linha.
    lcd.print("Temperatura:"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(0,1); //Posiciona o cursor do LCD na primeira coluna da segunda linha.
    lcd.print(temperature); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(5,1); //Posiciona o cursor do LCD na sexta coluna da segunda linha.
    lcd.print((char)223); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente (neste caso está-se a realizar print de "º").
    lcd.setCursor(6,1); //Posiciona o cursor do LCD na sétima coluna da segunda linha.
    lcd.print("C"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
  }
  if(watchHumidity){ //Verifica se é permitido expor o valor da humidade no LCD.
    lcd.setCursor(0,0); //Posiciona o cursor do LCD na primeira coluna da primeira linha.
    lcd.print("Humidade:"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(0,1); //Posiciona o cursor do LCD na primeira coluna da segunda linha.
    lcd.print(humidity); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(5,1); //Posiciona o cursor do LCD na sexta coluna da segunda linha.
    lcd.print("%"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
  }
}

void readLDR(){
  float ldr = analogRead(LDR); //Realiza a leitura da luminosidade captada pelo sensor.
  /*
  Mapeia os valores obtidos pelo sensor.
  Por norma, quando a luminosidade for máxima o sensor irá medir um valor de 0 e quando não houver luminosidade o sensor irá medir um valor de 4095.
  Esta função faz com que quando o sensor medir 0 o mesmo transmita um valor de 100 na plataforma e quando medir 4095 transmita 0 á plataforma, alterando as proporções dos valores intermédios para se adaptar aos limites definidos.
  */
  ldr = map(ldr,4095,0,0,100);
  Blynk.virtualWrite(V2, ldr); //Envia o valor da luminosidade lido pelo sensor para a plataforma Blynk, na qual a luminosidade foi associada ao pino virtual 2.
  //Realiza print do valor da leitura de luminosidade do sensor no Serial Monitor.
  Serial.print("Luminosity: ");
  Serial.print(ldr);
  Serial.println("%");
  if(watchLDR){ //Verifica se é permitido expor o valor da luminosidade no LCD.
    lcd.setCursor(0,0); //Posiciona o cursor do LCD na primeira coluna da primeira linha.
    lcd.print("Luminosidade:"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(0,1); //Posiciona o cursor do LCD na primeira coluna da segunda linha.
    lcd.print(ldr); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(5,1); //Posiciona o cursor do LCD na sexta coluna da segunda linha.
    lcd.print("%"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
  }
}

void readRain(){
  float rain = analogRead(Rain); //Realiza a leitura da intensidade da chuva captada pelo sensor.
  /*
  Mapeia os valores obtidos pelo sensor.
  Por norma, quando a intensidade da chuva for máxima o sensor irá medir um valor de 0 e quando não houver intensidade da chuva o sensor irá medir um valor de 4095.
  Esta função faz com que quando o sensor medir 0 o mesmo transmita um valor de 100 na plataforma e quando medir 4095 transmita 0 á plataforma, alterando as proporções dos valores intermédios para se adaptar aos limites definidos.
  */
  rain = map(rain,4095,0,0,100);
  Blynk.virtualWrite(V3, rain); //Envia o valor da intensidade da chuva lido pelo sensor para a plataforma Blynk, na qual a intensidade da chuva foi associada ao pino virtual 3.
  //Realiza print do valor da leitura da intensidade da chuva do sensor no Serial Monitor.
  Serial.print("Rain: ");
  Serial.print(rain);
  Serial.println("%");
  if(watchRain){ //Verifica se é permitido expor o valor da intensidade da chuva no LCD.
    lcd.setCursor(0,0); //Posiciona o cursor do LCD na primeira coluna da primeira linha.
    lcd.print("Chuva:"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(0,1); //Posiciona o cursor do LCD na primeira coluna da segunda linha.
    lcd.print(rain); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
    lcd.setCursor(5,1); //Posiciona o cursor do LCD na sexta coluna da segunda linha.
    lcd.print("%"); //Realiza print no LCD do parâmetro de entrada a partir da posição do cursor definida anteriormente.
  }
}