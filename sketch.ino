#include <WiFi.h>
#include <HTTPClient.h>

#define led_verde  2 // Pino utilizado para controle do led verde
#define led_vermelho  40 // Pino utilizado para controle do led vermelho
#define led_amarelo  9 // Pino utilizado para controle do led amarelo

#define ldr_D 4 // Pino utilizado para controle do led D
#define ldr_A 5 // Pino utilizado para controle do ldr A

#define botão_Pin 18 // Pino utilizado para controle do botão 
int estado_Botão = 0;  // Variável que faz a leitura do estado do Botão 

void setup() {
  Serial.begin(9600); //Configura Baud Route em 9600
  Serial.println("Sistema foi inicializado");
  digitalWrite(led_verde, led_vermelho, led_amarelo LOW); // Deixa todos leds desligados logo após inicialização 
  Serial.println("Todos Leds apagados");
  pinMode(led_verde, OUTPUT); // Definição inicial dos Leds como Outputs (Saídas)
  pinMode(led_vermelho, OUTPUT); // Definição inicial dos Leds como Outputs (Saídas)
  pinMode(led_amarelo, OUTPUT); // Definição inicial dos Leds como Outputs (Saídas)
}

//Cria Classe Noturno
class Noturno {
  public:
    Noturno();
    void piscarAmarelo();
  private:
    unsigned long piscar = 0;
    const unsigned long intervalo = 1000;
    bool ledOn = false;
};

Noturno::Noturno() {}

// Cria o método piscarAmarelo da classe Noturno
void Noturno::piscarAmarelo() {
// Pisca o led Amarelo de 1 em 1 segundos
unsigned long tempo = millis();

  if (tempo - piscar >= intervalo) {
    piscar = tempo;
    ledOn = !ledOn;
    digitalWrite(led_amarelo, ledOn ? HIGH : LOW);
  }
  digitalWrite(led_vermelho, led_verde LOW);
}

// Cria a classe Convencional 
class convencional {
    public:
        convencional();
        // Declara o método a ser construido
        void normal();
};
// Construtor da classe Day
convencional::convencional() {}

// Cria o método normal da classe convencional
void convencional: normal() {

  // Liga os leds de acordo o solicitado pela prova no modo convencional
    if (color == "verde") {
        digitalWrite(led_verde, HIGH);
        digitalWrite(led_amarelo, LOW);
        digitalWrite(led_vermelho, LOW);
        delay (3000);
    }
    if (color == "amarelo") {
        digitalWrite(led_verde, LOW);
        digitalWrite(led_amarelo, HIGH);
        digitalWrite(led_vermelho, LOW);
        delay (2000);
    }
    if (color == "vermelho") {
        digitalWrite(led_verde, LOW);
        digitalWrite(led_amarelo, LOW);
        digitalWrite(led_vermelho, HIGH);
        delay (5000);
    }
}

Noturno noturno;
Convencional convencional;

  // Inicialização das entradas
  pinMode(botão_Pin INPUT); // Configuração inicial dos pinos para controle do botão como INPUT (entrada) do ESP32
  pinMode(ldr_D INPUT); // Configuração inicial dos pinos para controle do LDR como INPUT (entrada) do ESP32
  pinMode(ldr_A INPUT); // Configuração inicial dos pinos para controle do LDR como INPUT (entrada) do ESP32

  Serial.begin(9600); // Configuração para debug por interface serial entre ESP e computador com baud rate de 9600

  WiFi.begin("Wokwi-GUEST", ""); // Conexão à rede WiFi aberta com SSID Wokwi-GUEST

  while (WiFi.status() != WL_CONNECT_FAILED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Conectado ao WiFi com sucesso!"); // Considerando que saiu do loop acima, o ESP32 agora está conectado ao WiFi (outra opção é colocar este comando dentro do if abaixo)


  if(WiFi.status() == WL_CONNECTED){ // Se o ESP32 estiver conectado à Internet
    HTTPClient http;

    String serverPath = "http://www.google.com.br/para fins de teste"; // Endpoint da requisição HTTP

    http.begin(serverPath.c_str());

    int httpResponseCode = http.GET(); // Código do Resultado da Requisição HTTP

    if (httpResponseCode>0) {
      Serial.print("Resposta do código HTTP: ");
      Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
      }
    else {
      Serial.print("Erro no código: ");
      Serial.println(httpResponseCode);
      }
      http.end();
    }

  else {
    Serial.println("WiFi Desconectado");
  }

void loop() {

  // Leitura do estado do botão
  int estado_Botão = digitalRead(botão_Pin);

  // Se o botão estiver pressionado 
  if (buttonState == HIGH) {
    digitalWrite(LED, LOW);
    Serial.println("Botão pressionado!");

  // Se o botão não estiver pressionado 
  } else {
    digitalWrite(LED, HIGH);
    Serial.println("Botão não pressionado!");

  }

  int ldrstatus = digitalRead(ldr_D);
  int ldrstatus = analogRead(ldr_A);

  if(ldrstatus<=threshold){
    Serial.print("Está escuro, led ligado");
    Serial.println(ldrstatus);

  }else{
    Serial.print("Está claro, led apagado");
    Serial.println(ldrstatus);
  }
}