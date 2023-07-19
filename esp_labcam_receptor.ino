
/*
  Estação Metereológica com o ESP32
  
  Compilador: Arduino 1.8.19

  Lora: receptor
  
*/

// ========== HABILITANDO MODO DEBUG ========
//#define DEBUG_BOTAO
#define DEBUG_SERIAL
#define DEBUG_MQTT
#define DEBUG_LORA
int lora_rssi = 0;

// ========== INCLUSAO DAS BIBLIOTECAS ========

#include<WiFi.h>                        // lib. para conexao WI-FI
#include<PubSubClient.h>                // lib. para conexao MQTT
#include<SPI.h>                         // lib. para barramento SPI
#include<string.h>

#ifdef DEBUG_LORA
  #include<LoRa.h>                      // lib. para conexao LoRa
  
#endif

// ========== Macros e constantes ==========
                                            // referentes ao sensor anemometro
#define PI_             3.14159265          // pi
#define PERIODO         5000                // intervalo de tempo para medicao em ms
#define INTAMOST        2000                // amostragem em ms
#define RAIOAN          147                 // raio do anemometro mm

                                            // referentes ao wi-fi
#define WIFI_SSID       "LABCAM"            // SSID da rede
#define WIFI_PASSWD     "labcam2020"            // senha da rede

                                            // referentes ao MQTT-Broker
#define MQTT_CLIENT     "EST_MET_ESP32C_RX"    // host como est. met. com esp32 ponta cliente
#define MQTT_USER       "labcam"            // usuario para conectar na rede MQTT
#define MQTT_PASSWD     "labcam2020"            // senha para conectar na rede MQTT
#define MQTT_SERVER     "200.239.165.51"    // endereco IP do servidor MQTT
#define MQTT_PORT       1883         
       // porta do servidor MQTT


#define MQTT_T_S_AN     "/est/senAnen"      // tópico MQTT sensor Ane.
#define MQTT_T_S_DV     "/est/senDven"      // tópico MQTT sensor dir. do vento
#define MQTT_T_S_SPR     "/est/senPR"
#define MQTT_T_S_SUM     "/est/senUM"
#define MQTT_T_S_TEMP     "/est/senTEMP"
#define MQTT_T_S_PLU     "/est/sensPLU"
#define MQTT_T_S_RAD     "/est/senRAD"
#define MQTT_T_S_RTC     "/est/senRTC"
#define MBUFFER         50                  // BUFFER para as mensagens 


#define LORA_GAIN       20                  // potencia do sinal em dBm
#define LORA_FREQ       915E6               // canal de op. em MHz
#define LORA_KEY_SINC   0xCA                // palavra para sincronizar rede LORA


                                            // estruturacao para tratamento de mensagens - SERIAL e MQTT
#define MID_DELIT        "/"                // delimitadores de mensagens recebida por barramentos
#define MID_UART          0                 // envio de mensagem recebida via uart - supervisorio trata as mensagens
#define MID_MQTT          1                 // envio de mensagem recebida via mqtt 
#define MID_F_OTH        "0"                // message id from group sensors
#define MID_F_SAN        "1"                // message id from sen. anem.
#define MID_F_SDV        "2"                // message id from sen. dir. v.
#define MID_F_SPR        "3"                // message id from sen. pressao
#define MID_F_SUM        "4"                // message id from sen. temp. e umid.
#define MQTT_T_S_TEM     "5"                // message id from sen. temp. e umid.
#define MID_F_SPLU       "6"                // message id from pluviômetro
#define MID_F_SRAD       "7"                // message id from radiance sensor
#define MID_F_RTC        "8"                // message id from RTC    



// ========== Mapeamento de portas =========

#define ESP_HELTEC


#ifdef ESP_HELTEC                               // mapeamento para ESP HELTEC LORA V2

    #define sensorAn    32                      // porta sensor anenometro
    #define sensorDv    12                      // porta sensor dir. do vento
                                                    // referentes ao barramento SPI e dispositivos - IMPORTANTE: conferir pinagem do SPI do kit usado
    #define SPI_SCK      5
    #define SPI_MISO    19
    #define SPI_MOSI    27
    
    #define LORA_SS     18
    #define LORA_RST    14
    #define LORA_DIO0   26
    
#else if ESP_DEVC                               // mapeamento para ESP DEV KIT C


    #define sensorAn    32                      // porta sensor anenometro
    #define sensorDv    12                      // porta sensor dir. do vento
                                                // referentes ao barramento SPI e dispositivos - IMPORTANTE: conferir pinagem do SPI do kit usado
    #define SPI_SCK     18
    #define SPI_MISO    19
    #define SPI_MOSI    23
    
    #define LORA_SS      5
    #define LORA_RST    16
    #define LORA_DIO0   17
#endif

// ========== Conf. de hardware ============



// ========== redefinicao de tipo ==========
typedef unsigned char u_int8;                                         // var. int. de  8 bits nao sinalizada
typedef unsigned char u_int16;                                        // var. int. de 16 bits nao sinalizada
typedef unsigned long u_int32;                                        // var. int. de 32 bits nao sinalizada


// ========== Variaveis globais ============
u_int32     contAn = 0,                                               // conta qtde de pulsos do sensor anemometro
            contAm = 0,                                               // contagem de tempo para intervalo entre amostras
            contPe = 0;                                               // contagem de tempo para intervalo de medicao
u_int8      tFAm   = 0,                                               // flag intervalo entre amostras / 1-finalizado
            tFPe   = 1,                                               // flag intervalo de medicao / 1-finalizado
            mFSAn   = 0;                                              // flag habilitando calculo de medicao pelo anemometro
char        message[MBUFFER],
            dirDv[MBUFFER];                                           // string para direcao do vento
float       speedAn = 0;                                              // variavel para armazenar velocidade do vento



// ========== Prototipos das Funcoes ========


u_int8 wifiConnect();                                                 // valida conexao do wi-fi
void mqttSetup();                                                     // valida conexao do mqtt
void mqttConnect();                                                   // funcao para conectar o MQTT
void callback(char* topic, byte* message, unsigned int length);
void    loraSetup();                                                  // configuracao da rede
u_int8  loraConnect(u_int32 freq);                                    // valida conexao do LORALORA
void    loraSendP(char *key, char *mesg);                             // realiza conexao LORA
u_int8 loraReceivedP(char *mensg);                                    // valida se recebeu mensagem na LORA
//void mensgProcess(u_int8 dest, char *mensg);                          // processa as mensagens recebidas
void IRAM_ATTR timer0_ISR();                                          // escopo de ISR do timer0
void IRAM_ATTR sensorAn_ISR();                                        // escopo de ISR do sensorAn

// ========== Declaracao dos objetos ========

WiFiClient espWFClient;                                               // objeto da classe WiFiClient
                                                                      // objeto pelo construtor da classe PubSubClient - MQTT    
PubSubClient client(MQTT_SERVER, MQTT_PORT, callback, espWFClient);   // callback: funcao que trata as mensagens recebidas                    
                                                        

// ========== Configuracoes iniciais ========
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  #ifdef DEBUG_MQTT
    wifiConnect();                                                    // chama a funcao que conecta ESP32 na rede Wi-Fi
    //mqttSetup();                                                      // chama a funcao que config. ESP32 na rede MQTT
                         
  #endif

    #ifdef DEBUG_LORA
    loraSetup();                                                      // chama a funcao que configura ESP32 na rede LORA
                         
  #endif
  

}

// ========== Codigo principal ==============
void loop() {
  // put your main code here, to run repeatedly:

    loraConnect(LORA_FREQ);                                             // chama a funcao que conecta ESP32 na rede MQTT e verifica se mantem conectado

        
    if(loraReceivedP(message)){       
                                // verifica se recebeu mensagem
  //      mensgProcess(MID_UART,message);                                  // processa mensagem para envio na UART
        
        //Serial.println(message);                                        // imprime na serial a mensagem recebida
        
        #ifdef DEBUG_MQTT
            wifiConnect();                                                    // chama a funcao que conecta ESP32 na rede Wi-Fi
            mqttConnect();                                                      // chama a funcao que conecta ESP32 na rede MQTT e verifica se mantem conectado
            mensgProcess(MID_MQTT,message);                                             // processa mensagem para envio via MQTT
            
        
        #endif
        
        
    }
    
                                    
}

// ========== Rotinas de interrupcao ========



// ========== Desenvolv. das funcoes ========



u_int8 wifiConnect(){

  static u_int8  wifiStatus = 0, statusChange = 0;                                  // status da conexao
  static u_int32 runtime = millis();                                                // tempo atual para controlar envio do log
  
  
  wifiStatus = ((WiFi.status() != WL_CONNECTED) ?  0 : 1);                           // se nao conectado, wifiStatus = 0

  
  
  if(!wifiStatus){

    WiFi.begin(WIFI_SSID, WIFI_PASSWD);                                             // passando o usuario e senha de rede wi-fi
    
if(!wifiStatus && (millis() - runtime > 1000)){                                     // mostra status da conexao a cada 1s
    
        Serial.println("[WiFi] Status: falha na conexao - nova tentativa...");
        runtime = millis();                                                         // atualiza temporizador
        statusChange = 0;                                                           // monitora se wi-fi mudou de estado
    }
    
  }
  else if((statusChange != wifiStatus)){                                            // se saiu de desconectado para conectado alerta
                                                                                    // se wi-fi conectou
    statusChange = wifiStatus;
    Serial.print("[WiFi] Status: Conectado - IP: ");
    Serial.println(WiFi.localIP());
    
  }
    
  return wifiStatus;                                                                // retorna status da conexao
        
}


                                                                          // configuracao do MQTT
void mqttSetup(){                                               
    client.setServer(MQTT_SERVER, MQTT_PORT);                             // definindo servidor e porta
    client.setCallback(callback);                                         // definindo a funcao que trata os dados recebidos da rede
    
}
                                                                          // funcao para conectar o MQTT
void mqttConnect() {


  if(!client.connected()){                                                // testa conexao do MQTT
      do {                                 
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASSWD)) {        // autenticao MQTT
          Serial.println("connected");
          // Subscribe
          // client.subscribe("esp32/output");                               // se increve no dominio mostrado
        } else {                                                          // caso contrario indica erro e tenta novamente
          Serial.print("failed, rc=");
          Serial.print(client.state());
          Serial.println(" try again in 5 seconds");
          // Wait 5 seconds before retrying
          delay(1000);
        }
      } while (!client.connected());
  }
  else{
    
    client.loop();                                                        // usada para continuar comunicacao com o servidor
  }
  
}
                                                                          // tratamento das mensagens recebidas
void callback(char* topic, byte* message, unsigned int length){

}

#ifdef DEBUG_LORA

    void loraSetup(){
 
      Serial.println("[LoRa] Configuracao LoRa...");                                    // indica tentativa de fechar enlace Lora
      
      LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);                                       // associando os pinos do SPI com o radio LORA

     LoRa.setSyncWord(LORA_KEY_SINC);                                                           // palavra para sincronizar e receber mensagens na rede
                                                                                        // necessario no TX e RX
      Serial.println("[LoRa] Configurado");
              
      LoRa.setTxPower(LORA_GAIN);                                                       // seta ganho do radio LoRa - 20dBm é o máximo
      
    }
                                                                                        // habilita conexao LoRa
    u_int8 loraConnect(u_int32 freq){                                                  

      static u_int8  loraStatus = 0;                                                    // status da conexao
      static u_int32 runtime = millis();                                                // tempo atual para controlar envio do log
                    
      
            
      if(!loraStatus){                                                             
        
        if(!LoRa.begin(LORA_FREQ)){                                                     // verifica se habilitou para conexao 
        
            
            loraStatus = 0;                                                             // falha na conexao
                
            if(millis() - runtime > 1000){                                              // mostra status da conexao a cada 1s
        
                Serial.println("[LoRa] Status: falha na inicializacao - nova tentativa... ");
                runtime = millis();                                                     // atualiza temporizador
                
            }
            
        }
        else{
            Serial.println("[LoRa] Status: modulo inicializado ");
            loraStatus = 1;                                                             // sucesso na conexao
                
        }

            
      }

      return loraStatus;                                                                // retorna status
      
    }
    
   
                                                                                
    u_int8 loraReceivedP(char *mensg){                                            // funcao para recebimento de pacotes pela LORA


      u_int8  mensgStatus = 0;                                                    // status se recebeu mensagem
      String  pacRec;                                                             // pacote recebido
      u_int32 pacSize = 0;                                                        // tamanho do pac.
      

        if(loraConnect(LORA_FREQ)){
  
          pacSize = LoRa.parsePacket();   
          

          if(pacSize){                                                            // caso tenha recibido pacote
    
            mensgStatus = 1;                                                      // recebeu mensagem
          
            //Serial.print("Mensagem recebida: ");
    
            while(LoRa.available()){                                              // recebe mensagem enquanto tiver dados disponiveis
                pacRec = LoRa.readString();
                //Serial.print(pacRec);
            }
    
            strcpy(mensg, pacRec.c_str());                                        // passando mensagem para var. enviada como parametro
    
          }
          else{
            //Serial.println("Sem mensagem recebida ");
            mensgStatus = 0;                                                      // nao recebeu mensagem
          }
      
        }
  
        
        return mensgStatus;                                                       // retorna se recebeu ou nao uma mensagem
      
    }
    
#endif
                                                                                    // processa as mensagens recebidas na rede LORA
                                                                                    // dest: UART ou MQTT 
    void mensgProcess(u_int8 dest, char *mensg){
                                                                                    /*
                                                                                       * escopo das mensagens para UART
                                                                                       *    delimitador de string "/"
                                                                                       *    [mensg. sens.   ]: ex.: "33RPM" / tratamento via supervisorio
                                                                                       *    
                                                                                       *    escopo das mensagens para MQTT
                                                                                       *    delimitador de string "/"
                                                                                       *    [orig. sens.    ]
                                                                                       *    [valor sens.    ]: ex.: 33/0 como 33 - 0/RPM
                                                                                       *    
                                                                                    */
    
         
        char    *breakMesg,
                newMesg[9][MBUFFER] = {" ", " ", " ", " ", " "};                    // matriz de strings
                u_int8 i = 0;                                                       // var. aux. de indices
        lora_rssi = LoRa.packetRssi(); 
        char rssi[9];                                         
        sprintf (rssi, "%d", lora_rssi);
    
        if(dest == MID_UART){                                                       // envio de mensagem via serial/UART
            Serial.println(mensg);
        }
        else if(dest == MID_MQTT){                                                  // envio de mensagem via MQTT
            
            breakMesg = strtok(mensg, MID_DELIT);                                   // atribuindo ao conteudo a mensagem separada pelo primeiro delimitador
            while(breakMesg != NULL){                                               // enquanto ponteiro nao apontar NULL - vazio
                //Serial.println(breakMesg);
                strcpy(newMesg[i], breakMesg);                                      // copiando string delimitada atual na posicao i
                breakMesg = strtok(NULL, MID_DELIT);                                // quebra mensagem incrementa ponteiro                                
                i++;                                                                // incrementando indice para acompanhar indices
            }
            
            if(!strcmp(newMesg[0],MID_F_SAN)){
                client.publish(MQTT_T_S_AN, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
            else if (!strcmp(newMesg[0],MID_F_SDV)){
                client.publish(MQTT_T_S_DV, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
            else if (!strcmp(newMesg[0],MID_F_SPR)){
                client.publish(MQTT_T_S_SPR, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
             else if (!strcmp(newMesg[0],MID_F_SUM)){
                client.publish(MQTT_T_S_SUM, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
             else if (!strcmp(newMesg[0],MQTT_T_S_TEM)){
                client.publish(MQTT_T_S_TEMP, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
             else if (!strcmp(newMesg[0],MID_F_SPLU)){
                client.publish(MQTT_T_S_PLU, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
            else if (!strcmp(newMesg[0],MID_F_SRAD)){
                client.publish(MQTT_T_S_RAD, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
            else if (!strcmp(newMesg[0],MID_F_RTC)){
                client.publish(MQTT_T_S_RTC, newMesg[1]);                            // escrevendo valor no top. mqtt - se = 0, strings iguais
            }
           
           client.publish("/est/rssi",rssi);
           //Serial.println(rssi);

            

    
        }
        
    }
   
