

// ========== HABILITANDO MODO DEBUG ========
//#define DEBUG_BOTAO
//#define DEBUG_SERIAL
//#define DEBUG_MQTT
#define DEBUG_LORA

// ========== INCLUSAO DAS BIBLIOTECAS ========



#include <SPI.h>           // lib. para barramento SPI
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> // lib. para o BME280


//cartao
#include "FS.h"  // libs. para o cartão SD
#include "SD.h"
#include "SPI.h"
#include <time.h>
SPIClass spi1;

#include <DS1307ESP.h> // libs. para o RTC

#include <vector>
using std::vector;
#include <iterator>
using std::iterator;

#ifdef DEBUG_LORA
#include <LoRa.h>  // lib. para conexao LoRa

#endif
#include <string.h>  // lib. para manipular strings

// ========== Macros e constantes ==========

#define PI_ 3.14159265           // pi
#define PERIODO 250              // intervalo de tempo para medicao em ms - Amostras a cada 250ms
#define N_AMOSTRAS1MIN  12
#define INIT_MIN  99E10
#define INIT_MAX  -99E10
#define RAIO_AN 147E-3           // raio do anemometro mm
#define K_AN 3.14                // coef. admensional do anemometro
#define SEALEVELPRESSURE_HPA (1013.25)
#define ATTACH_INT   attachInterrupt(digitalPinToInterrupt(sensorAn), sensorAn_ISR, FALLING);   attachInterrupt(digitalPinToInterrupt(sensorPluv), sensorPluv_ISR, RISING);
#define DETACH_INT   detachInterrupt(digitalPinToInterrupt(sensorAn));  detachInterrupt(digitalPinToInterrupt(sensorPluv));




#define MBUFFER 10                    // BUFFER para as mensagens

// referentes ao LoRa
#define LORA_GAIN 20        // potencia do sinal em dBm
#define LORA_FREQ 915E6     // canal de op. em MHz
#define LORA_KEY_SINC 0xCA  // palavra para sincronizar rede LORA

// estruturacao para tratamento de mensagens - SERIAL e MQTT
#define MID_DELIT "/"               // delimitadores de mensagens recebida por barramentos
#define MID_UART 0                  // envio de mensagem recebida via uart - supervisorio trata as mensagens
#define MID_MQTT 1                  // envio de mensagem recebida via mqtt
#define MID_F_OTH "0"               // message id from group sensors
#define MID_F_SAN "1"               // message id from sen. anem.
#define MID_F_SDV "2"               // message id from sen. dir. v.
#define MID_F_SPR "3"               // message id from sen. pressao
#define MID_F_SUM "4"               // message id from sen. temp. e umid.
#define MID_F_STE "5"               // message id from sen. temp. e umid.
#define MID_F_SPLU "6"              // message id from pluviômetro
#define MID_F_SRAD "7"              // message id from radiance sensor
#define MID_F_RTC "8"               // message id from RTC



// ========== Mapeamento de portas =========

#define ESP_HELTEC
//#define ESP_DEVC

#ifdef ESP_HELTEC  // mapeamento para ESP HELTEC LORA V2

#define sensorAn 12    // porta sensor anenometro
#define sensorDv 36    // porta sensor dir. do vento
#define sensorPluv 13  // porta sensor pluviometro
#define sensor_irad 34

// referentes ao barramento SPI e dispositivos - IMPORTANTE: conferir pinagem do SPI do kit usado
#define SPI_SCK 17
#define SPI_MISO 32
#define SPI_MOSI 2

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

#define SD_CS 4  // Define CS pin for the SD card module


#endif


const char *csvHeader = "NOME DA ESTAÇÃO,BAT(%), ANO, MÊS, DIA,HORA, MINUTO,TEMP_INST,TEMP_ MAX(ºC),TEMP_MIN(ºC), UMID_INST(%),UMID_MAX(%),UMID_MIN(%),PRESSÃO_INST(hPa),PRESSÃO_MAX(hPa),PRESSÃO_MIN(hPa), VENTO_VEL(m/s),VENTO_DIR(º),RAD(kJm2),PREC(mm)\n";
typedef struct {
  float temp, umid, pres, vento, rad;
  uint8_t bat;
  uint16_t ano, mes, dia, hora, minuto, vento_dir;
  float vento_raj, prec;
} WeatherData;



// ========== Conf. de hardware ============
             
hw_timer_t *timer0 = NULL;              // declarando uso de timer via hardware
//hw_timer_t * timerBegin(uint8_t num, uint16_t divider, bool countUp);

// ========== Variaveis globais ============
uint32_t cont_anemometro = 0,  // conta qtde de pulsos do sensor anemometro
  contIntervAmostras = 0,      // contagem de tempo para intervalo entre amostras
  cont_periodo_read_an = 0,    // contagem de tempo para intervalo de medicao (PERIODO_READ_AN para coleta de pulsos do anemometro)
  cont_periodo_read_pluv = 0,
  cont_periodo_read_analog_sensors = 0,
  cont_periodo_salvar_cartao = 0,
  cont_pluviometro = 0,
  cont_envio = 0,  // contagem dos segundos para período de envio das mensagens
  lastTime;

uint8_t flagIntervAmostras = 0,  // flag intervalo entre amostras / 1-finalizado
  sdapin = 21,
        sclpin = 22,
        irradiance = 0,
        flagIntervMedicao = 1,   // flag intervalo de medicao / 1-finalizado
  flag_calculo_pluviometro = 0,  // flag para garantir que o cálculo do volume de chuva só é
                                 // realizado após 2x o tempo de amostragem do anemômetro (2 períodos de medição)
  flag_calculo_anemometro = 0,   // flag habilitando calculo de medicao pelo anemometro
  flag_envio = 0,                // flag para habilitar envio de mensagens
  //flag_interrupcao = 1,
  cont_first = 0,
        amostras1s = 0,
        flag_periodo_leituras_analogicas = 0,
        flag_periodo_salvar_cartao = 0,
        cont_amostras1min = 0;
const char *csvFilePath = "/dados.csv";
char message[MBUFFER];
double volume_chuva = 0;
float speed_an = 0,  // variavel para armazenar velocidade do vento
        // variável para armazenar volume de chuva
      
      media_irad = 0,
      soma_irad = 0,
      max_irad = 0,
      min_irad = 0,
      
      media_humidity = 0,
      soma_humidity = 0,
      max_humidity = INIT_MAX,
      min_humidity = INIT_MIN,
      
      media_temperature = 0,
      soma_temperature = 0,
      max_temperature = INIT_MAX,
      min_temperature = INIT_MIN,
      
      media_pressure = 0,
      soma_pressure = 0,
      max_pressure = INIT_MAX,
      min_pressure = INIT_MIN,
      
      media_dir_vento = -99,
      media_vel_vento = -99;

int a = 0;

String dataMessage,  //variavel que ira armazenar os dados no cartao SD
  time_rtc;

vector<float> vel_vento3s;  // Vector container do tipo inteiro para armazenar as leituras do anemômetro durante 3s
vector<float> dir_vento3s;  // Vector container do tipo inteiro para armazenar as leituras de direção do vento durante 3s


// definicao de estruturas
enum unidV { RPM,
             M_S,
             KM_H };  // estrutura com unidades de velocidade do vento

// ========== Prototipos das Funcoes ========
float avg(vector<float> v);                                      // realiza média em vectors
float speedV(uint8_t);                                           // calculo da velocidade do vento
float volumeChuva();                                             // calcula o volume de chuva de acordo com o pluviômetro
void leiturasVarAnalogicas();                                    // Realiza a leitura das variáveis analógicas
uint16_t dirVento();                                             // leitura de sensor de dir. do vento
void loraSetup();                                                // configuracao da rede
uint8_t loraConnect(uint32_t freq);                              // valida conexao do LORALORA
void loraSendP(char *key, char *mesg);                           // realiza conexao LORA
void createCSVFile(fs::FS &fs, const char *path);
void appendFile(fs::FS &fs, const char *path, const char *message);
int monthNameToNumber();
void mediaVelDirVento();
void IRAM_ATTR timer0_ISR();      // escopo de ISR do timer0
void IRAM_ATTR sensorAn_ISR();    // escopo de ISR do sensorAn
void IRAM_ATTR sensorPluv_ISR();  // escopo de ISR do Pluviometro

// ========== Declaracao dos objetos ========

Adafruit_BME280 bme;                                                 // I2C                                           // BME280 object
DS1307ESP rtc;                                                       // DS1307 object
                                                                     // objeto pelo construtor da classe PubSubClient - MQTT

//void onTimer();

#ifdef DEBUG_SERIAL
uint32_t aux;  // var aux debug - remover
#endif
#ifdef DEBUG_BOTAO
uint32_t aux2;  // var aux debug - remover
#endif

// ========== Configuracoes iniciais ========
void setup() {

  Serial.begin(9600);

  loraSetup();  // chama a funcao que configura ESP32 na rede LORA
                // configuracao inicial das entradas e saidas

  pinMode(sensorAn, INPUT_PULLUP);    // porta em resistive pull up usando resistores internos
  pinMode(sensorPluv, INPUT_PULLUP);  // porta em resistive pull up usando resistores internos
 SPIClass(1);
  spi1.begin(17, 32, 2, 4);
  

  if (!SD.begin(4, spi1)) {
    Serial.println("Card Mount Failed");
  }
  bme.begin(0x76);  
  rtc.begin(sdapin, sclpin);               //sdapin 21 , sclpin 22
  rtc.DSadjust(14, 00, 00, 2023, 7, 12);  // 00:19:21 16 Mar 2022
//  rtc.SetFont(0);                        //  language 0 = EN  |  1 = FR  |  2 = GR

  timer0 = timerBegin(0, 80, true);                 // inicializando timer 0 com prescalaler 1:80 - fcpu = 80Mhz
  timerAttachInterrupt(timer0, &timer0_ISR, true);  // configurando inter. do timer 0
  timerAlarmWrite(timer0, 1000, true);              // ISR a cada 1ms
  timerAlarmEnable(timer0);                         // ativa interrupcao pelo timer 0

  ATTACH_INT
}

// ========== Codigo principal ==============
void loop() {

  if (flag_calculo_anemometro) {
    DETACH_INT
    mediaVelDirVento();
   // Serial.print("Media de velocidade e direcao do vento: ");
    //Serial.println(media_vel_vento);
    //Serial.println(media_dir_vento);
    ATTACH_INT
  }

  if (flag_calculo_pluviometro) {
    DETACH_INT
    volume_chuva += volumeChuva();
//    Serial.println("Volume de chuva: ");
//    Serial.println(volume_chuva);
    ATTACH_INT
  }

  if (flag_periodo_leituras_analogicas) {
    DETACH_INT
    rtc.DSread();
    time_rtc = rtc.getTimeDate();  //  (String) 12:07:18 Mon, 14 Mar 2022
    leiturasVarAnalogicas();
    //Serial.println("Media temperatura: ");
    //Serial.println(media_temperature);   
    ATTACH_INT
  }

  if (flag_envio) {
    DETACH_INT
    //Serial.println("Sending...");
    // envia pacote na rede LORA
    snprintf(message, MBUFFER, "%.2f", media_vel_vento);
    Serial.print("media_vel_vento: ");
    Serial.println(media_vel_vento);
    
    loraSendP(MID_F_SAN, message);
    //Serial.println("---===============+++++++++++++++++++++----------------------------------------------");
    //Serial.println(media_dir_vento);
    snprintf(message, MBUFFER, "%.2f", media_dir_vento);
    Serial.println("--------------------------------------------------------");
    Serial.println(message);
    loraSendP(MID_F_SDV, message);

    snprintf(message, MBUFFER, "%.2f", media_pressure);
    //Serial.print("media_pressure: ");
    //Serial.println(media_pressure);
    loraSendP(MID_F_SPR, message);

    snprintf(message, MBUFFER, "%.2f", media_humidity);
    //Serial.print("Humidade: ");
    //Serial.println(media_humidity);
    loraSendP(MID_F_SUM, message);

    snprintf(message, MBUFFER, "%.2f", media_temperature);
    //Serial.print("media_temperature: ");
    //Serial.println(media_temperature);
    loraSendP(MID_F_STE, message);

    snprintf(message, MBUFFER, "%.2f", volume_chuva);
    loraSendP(MID_F_SPLU, message);

    snprintf(message, MBUFFER, "%.2f", media_irad);
    loraSendP(MID_F_SRAD, message);

    snprintf(message, MBUFFER, "%s", rtc.getTime());
    loraSendP(MID_F_RTC, message);

    cont_envio = 0;
    flag_envio = 0;

    ATTACH_INT
  }
  if (flag_periodo_salvar_cartao) {
    DETACH_INT
    if (!fileExists(SD, csvFilePath)) {
      createCSVFile(SD, csvFilePath);
    } else {
                       // NOME DA ESTAÇÃO,BAT(%), ANO, MÊS, DIA,HORA, MINUTO,                                                                                                                           TEMP_INST,TEMP_ MAX(ºC),TEMP_MIN(ºC), UMID_INST(%),UMID_MAX(%),UMID_MIN(%) PRESSÃO_INST(hPa),PRESSÃO_MAX(hPa),PRESSÃO_MIN(hPa), VENTO_VEL(m/s),VENTO_DIR(º),VENTO_RAJ(m/s),RAD(kJm2),PREC(mm)
      
      String csvData = "UFOP_1001,bateria," + String(rtc.getYear()) + "," + String(monthNameToNumber()) + "," + String(rtc.getDay()) + "," + String(rtc.getHour()) + "," + String(rtc.getMinute()) + "," + String(media_temperature) + "," + String(max_temperature) + "," + String(min_temperature) + "," + String(media_humidity) + "," + String(max_humidity) + "," + String(min_humidity) + "," + String(media_pressure) + "," + String(max_pressure) + "," + String(min_pressure) + "," + String(media_vel_vento) +  "," + String(media_dir_vento) + "," + String(media_irad) + "," + String(volume_chuva) + "\n";

      appendFile(SD, "/dados.csv", csvData.c_str());
    }

  Serial.println("rtc.getYear(): " + String(rtc.getYear()));
  Serial.println("monthNameToNumber(): " + String(monthNameToNumber()));
  Serial.println("rtc.getDay(): " + String(rtc.getDay()));
  Serial.println("rtc.getHour(): " + String(rtc.getHour()));
  Serial.println("rtc.getMinute(): " + String(rtc.getMinute()));
  Serial.println("media_temperature: " + String(media_temperature));
  Serial.println("max_temperature: " + String(max_temperature));
  Serial.println("min_temperature: " + String(min_temperature));
  Serial.println("media_humidity: " + String(media_humidity));
  Serial.println("max_humidity: " + String(max_humidity));
  Serial.println("min_humidity: " + String(min_humidity));
  Serial.println("media_pressure: " + String(media_pressure));
  Serial.println("max_pressure: " + String(max_pressure));
  Serial.println("min_pressure: " + String(min_pressure));
  Serial.println("media_vel_vento: " + String(media_vel_vento));
  Serial.println("media_dir_vento: " + String(media_dir_vento));
  Serial.println("media_irad: " + String(media_irad));
  Serial.println("volume_chuva: " + String(volume_chuva));
    flag_periodo_salvar_cartao = 0;
    cont_periodo_salvar_cartao = 0;
    ATTACH_INT
  }

#ifdef DEBUG_LORA
  loraConnect(LORA_FREQ);  // chama a funcao que conecta ESP32 na rede MQTT e verifica se mantem conectado
#endif


#ifdef DEBUG_MQTT
  mqttConnect();  // chama a funcao que conecta ESP32 na rede MQTT e verifica se mantem conectado
#endif
}

// ========== Rotinas de interrupcao ========

void IRAM_ATTR sensorAn_ISR() {
  cont_anemometro++;  // registra quantidade de pulsos - voltas completas
}

void IRAM_ATTR sensorPluv_ISR() {
  // contando a cada borda de descida do reed switch
  // dinâmica do reed switch a cada pulso: low-high-low
  cont_pluviometro++;
}

void IRAM_ATTR timer0_ISR() {  // ISR Handler timer0 - 1ms

  if (cont_periodo_read_an < PERIODO) { // 250ms
    cont_periodo_read_an++;
    flag_calculo_anemometro = 0;
  } else if (cont_periodo_read_an == PERIODO) {
    flag_calculo_anemometro = 1;
  }

  if (cont_periodo_read_pluv < (40 * PERIODO)) {  // 40 * 0.25s = 10s
    cont_periodo_read_pluv++;
    flag_calculo_pluviometro = 0;
  } else if (cont_periodo_read_pluv == (40 * PERIODO)) {
    flag_calculo_pluviometro = 1;
  }

  if (cont_periodo_read_analog_sensors < (20 * PERIODO)) { // 20 * 0.25s = 5s
    cont_periodo_read_analog_sensors++;
    flag_periodo_leituras_analogicas = 0;
  } else if (cont_periodo_read_analog_sensors == (20 * PERIODO)) {
    flag_periodo_leituras_analogicas = 1;
  }

  if (cont_envio < (PERIODO * 120 )) {  // Verifica se é a hora de enviar as mensagens (a cada 1 min)
    cont_envio++;
    flag_envio = 0;
  } else if (cont_envio == (PERIODO * 120 )) {
    flag_envio = 1;
  }
  if (cont_periodo_salvar_cartao < (240 * PERIODO)) {
    cont_periodo_salvar_cartao++;
    flag_periodo_salvar_cartao = 0;
  } else if (cont_periodo_salvar_cartao == (240 * PERIODO)) {
    flag_periodo_salvar_cartao = 1;
  }
}


// ========== Desenvolv. das funcoes ========

float avg(vector<float> v) {  // média dos valores de pulso medidos pelo anemômetro no período de 3s
  uint16_t sum = 0;
  for (vector<float>::iterator it = v.begin(); it != v.end(); it++) {
    sum += *it;
  }
  return (sum / v.size());
}

void leiturasVarAnalogicas() {
  float rad,
        humidity,
        temperature,
        pressure;
  if(cont_amostras1min < 12){
    rad = irad();
    humidity = bme.readHumidity(); // %
    temperature = bme.readTemperature();// *C
    pressure = bme.readPressure()/ 100.0F; // hPa

    soma_irad+=rad;
    soma_humidity+=humidity;
    soma_temperature+=temperature;
    soma_pressure+=pressure;

    if(rad > max_irad) max_irad = rad;
    if(humidity > max_humidity) max_humidity = humidity;
    if(temperature > max_temperature) max_temperature = temperature;
    if(pressure > max_pressure) max_pressure = pressure;

    if(rad < min_irad) min_irad = rad;
    if(humidity < min_humidity) min_humidity = humidity;
    if(temperature < min_temperature) min_temperature = temperature;
    if(pressure < min_pressure) min_pressure = pressure;

    cont_amostras1min++;

  } else {
    media_irad = soma_irad/N_AMOSTRAS1MIN;
    media_humidity = soma_humidity/N_AMOSTRAS1MIN;
    media_temperature = soma_temperature/N_AMOSTRAS1MIN;
    media_pressure = soma_pressure/N_AMOSTRAS1MIN;
    cont_amostras1min = 0;
    soma_irad = 0;
    soma_humidity = 0;
    soma_temperature = 0;
    soma_pressure = 0;
    max_irad = INIT_MAX;
    max_humidity = INIT_MAX;
    max_temperature = INIT_MAX;
    max_pressure = INIT_MAX;
    min_irad = INIT_MIN;
    min_humidity = INIT_MIN;
    min_temperature = INIT_MIN;
    min_pressure = INIT_MIN;
  }
  cont_periodo_read_analog_sensors = 0;
  flag_periodo_leituras_analogicas = 0;
}

void mediaVelDirVento() {

  if (vel_vento3s.size() < 12) {  // Só vai para o else depois de 3s (12 * 0.25)
    vel_vento3s.push_back(speedV(M_S));
    dir_vento3s.push_back(dirVento());
    cont_first++;
  } else if (cont_first == 12) {
    media_vel_vento = avg(vel_vento3s);
    media_dir_vento = avg(dir_vento3s);
    cont_first = 0;
  } else {  // sempre entrará nesse laço após os 3s iniciais
    vel_vento3s.erase(vel_vento3s.begin());
    vel_vento3s.push_back(speedV(M_S));
    dir_vento3s.erase(dir_vento3s.begin());
    dir_vento3s.push_back(dirVento());
    amostras1s++;
    if (amostras1s == 4) {
      media_vel_vento = avg(vel_vento3s);
      media_dir_vento = avg(dir_vento3s);
      amostras1s = 0;
    }
  }
  cont_periodo_read_an = 0;
  flag_calculo_anemometro = 0;
}

float volumeChuva() {
  float volume = 0.0;
  static uint32_t old_cont = 0;
  volume = 0.25 * (cont_pluviometro - old_cont);
  old_cont = cont_pluviometro;
  flag_calculo_pluviometro = 0;
  cont_periodo_read_pluv = 0;
  return (volume);
}

float speedV(uint8_t op) {  // calcula velocidade do vento

  // var. auxiar do vento em velocidade em RPS
  float spv = (cont_anemometro / (PERIODO / 1000.0));
  switch (op) {
    case RPM: spv = spv * 60; break;                                // velocidade em RPM por padrao
    case M_S: spv = 2.0 * PI * spv * K_AN * (RAIO_AN); break;       // velocidade em m/s
    case KM_H: spv = 2.0 * PI * spv * K_AN * (RAIO_AN)*3.6; break;  // velocidade em km/h
  }
  cont_anemometro = 0;  // reseta contador de voltas
  return spv;           // retornando velocidade
}

//void dirVento(char *sDir){
uint16_t dirVento() {

  float valor = 0;       // var para leitura do sensor
  uint16_t Winddir = 0;  // dir. do vento por graus
  //String sDir;


  valor = analogRead(sensorDv) * (3.3 / 4095);  // leitura ADC do sensor

  //Serial.print("\n\n\t\tleitura do sensor: ");
  //Serial.print(valor);
  //Serial.print(" volt");


  if (valor <= 0.37) {  // 315° - Noroeste
    Winddir = 315;
    //sDir = " noroeste";
    //strcpy(sDir," noroeste");
  } else if (valor <= 0.41) {  // 270° - Oeste
    Winddir = 270;
    //sDir = " oeste";
    //strcpy(sDir," oeste");
  } else if (valor <= 0.47) {  // 225° - Sudoeste
    Winddir = 225;
    //sDir = " sudeste";
    //strcpy(sDir," sudeste");
  } else if (valor <= 0.55) {  // 180° - Sul
    Winddir = 180;
    //sDir = " sul";
    //strcpy(sDir," sul");
  } else if (valor <= 0.66) {  // 135° - Sudeste
    Winddir = 135;
    //sDir = " sudeste";
    //strcpy(sDir," sudeste");
  } else if (valor <= 0.83) {  // 90° - Leste
    Winddir = 90;
    //sDir = " leste";
    //strcpy(sDir," leste");
  } else if (valor <= 1.10) {  // 45° - Nordeste
    Winddir = 45;
    //sDir = " nordeste";
    //strcpy(sDir," nordeste");
  } else {
    Winddir = 0;  // 0° - Norte
    //sDir = " norte";
    //strcpy(sDir," norte");
  }

  //Serial.print("\n\n\t\tDirecao a :");
  //Serial.print(Winddir);

  return Winddir;
}

float irad() {
  int adc_value = analogRead(sensor_irad);

  // Converte o valor lido para tensão (intervalo de 0V a 3.3V) e formata como string
  float irrad = ((adc_value)*5 / (4095.0 * 6)) * 3.3;
  Serial.print("Rad:"); 
  Serial.println(irrad);

  return irrad;
}

// funcao para conectar o wireless



#ifdef DEBUG_LORA
// configura rede LORA
void loraSetup() {

  Serial.println("[LoRa] Configurando o radio LoRa...");  // indica tentativa de fechar enlace Lora
 
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);  // associando os pinos do SPI com o radio LORA

  LoRa.setSyncWord(LORA_KEY_SINC);  // palavra para sincronizar e receber mensagens na rede
                                    // necessario no TX e RX
  Serial.println("[LoRa] Configurado");

  LoRa.setTxPower(LORA_GAIN);  // seta ganho do radio LoRa - 20dBm é o máximo
}
// conexao LoRa
uint8_t loraConnect(uint32_t freq) {

  static uint8_t loraStatus = 0;       // status da conexao
  static uint32_t runtime = millis();  // tempo atual para controlar envio do log



  if (!loraStatus) {

    if (!LoRa.begin(LORA_FREQ)) {  // verifica se habilitou para conexao


      loraStatus = 0;  // falha na conexao

      if (millis() - runtime > 1000) {  // mostra status da conexao a cada 1s

        Serial.println("[LoRa] Status: falha na inicializacao - nova tentativa... ");
        runtime = millis();  // atualiza temporizador
      }

    } else {

      Serial.println("[LoRa] Status: modulo inicializado ");
      loraStatus = 1;  // sucesso na conexao
    }
  }

  return loraStatus;  // retorna status
}
//rtc
int monthNameToNumber() {
  String month = rtc.getMonth(true);  // Suponha que isso retorne o nome abreviado do mês

  int monthNumber = 0;
  String monthNames[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

  for (int i = 0; i < 12; i++) {
    if (month == monthNames[i]) {
      monthNumber = i + 1;
      break;
    }
  }

  return monthNumber;
}
//cartao
bool fileExists(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  if (!file) {
    return false;
  }
}
void createCSVFile(fs::FS &fs, const char *path) {
  Serial.printf("[SD Card] criando o cabeçalho do arquivo: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("[SD Card] Falha ao abrir o arquivo para a escrita ...");
   // return;
  }

  if (file.print(csvHeader)) {
    Serial.println("[SD Card] Escrevendo o cabeçalho ...");
  } else {
    Serial.println("[SD Card] Falha ao escrever o cabeçalho ...");
  }

  file.close();
}
void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("[SD Card] Falha ao abrir o arquivo ...");
    return;
  }
  if (file.print(message)) {
    Serial.println("[SD Card] menssagem adicionada ...");
  } else {
    Serial.println("[SD Card] Falha ao adicionar a menssagem");
  }
  file.close();
}

void loraSendP(char *key, char *mesg) {  // funcao para envio de pacotes pela LORA

  if (loraConnect(LORA_FREQ)) {
    LoRa.beginPacket();     // inicio de pacote LORA
    LoRa.print(key);        // enviando chave de de tipo e origem das mensagens
    LoRa.print(MID_DELIT);  // enviando delimitador entre informacoes
    LoRa.print(mesg);       // corpo da mensagem
    LoRa.endPacket();       // finalizando envio de pacote LORA
    //Serial.println("[LoRa] Status: mensagem enviada");
  } else {
    Serial.println("[LoRa] Status: Erro no envio");
  }
}

#endif

/*
 * INFORMACOES ANEMOMETRO
 * https://www.usinainfo.com.br/blog/anemometro-arduino-um-sensor-de-vento-para-estacao-meteorologica/
 * 
 * IMPLEMENTAR TIMER COM INTERRUPÇÃO
 * https://www.dobitaobyte.com.br/timer-com-esp8266-na-ide-do-arduino/
 * 
 * MQTT https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
 * BASE: /home/labcam/Arduino/esp01_arduino/esp01_arduino.ino
 * 
 * LORA
 * https://www.filipeflop.com/blog/comunicacao-lora-ponto-a-ponto-com-modulos-esp32-lora/
 * https://github.com/sandeepmistry/arduino-LoRa#readme
 * https://www.fernandok.com/2019/03/instalacao-do-esp32-lora-na-arduino-ide.html * 
 * https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md
 * https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/
 * https://embarcados.com.br/esp32-adc-interno/
 */
