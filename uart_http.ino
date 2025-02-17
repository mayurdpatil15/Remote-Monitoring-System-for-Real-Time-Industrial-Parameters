

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>

// WiFi configuration
const char* ssid = "Spam";
const char* password = "7218824546";

// ThingSpeak configuration
const char* server = "http://api.thingspeak.com"; //ThinkSpeak server
const String apiKey = "RYONUM534IL78R6S"; // ThinkSpeak API key

//Serial monitor usart Rx, Tx
#define RX_PIN D1  // Connect this to the TX pin of STM32
#define TX_PIN D2  // Connect this to the RX pin of STM32
SoftwareSerial mySerial(RX_PIN, TX_PIN);


void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  Serial.println();

  pinMode(D0, OUTPUT);
  mySerial.begin(9600); //baudrate equal to stm32
  digitalWrite(D0, HIGH);
  Serial.println("Setup completed.");
  // Connect to Wi-Fi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
 }

void loop() {
  
    digitalWrite(D0, HIGH);
    // mySerial.write('1');
   if (mySerial.available() > 0) {     
    digitalWrite(D0, LOW);                                   //check data on serail monitor from stm32
    String receivedData = mySerial.readStringUntil('\r');               //read until "/r"
    receivedData.trim(); // Remove any extra whitespace 
    Serial.print("Message from STM32: ");
    Serial.println(receivedData); // Print the received message to Serial Monitor
    Serial.print("Length of the received string: ");
    int length = receivedData.length();
    Serial.println(length);
    // delay(1000);
    //THingspeak code (WiFi)

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client; // Create a WiFi client
    HTTPClient http;   // Create an HTTP client

    // Generate the URL to send data
    String url = String(server) + "/update?api_key=" + apiKey + String(receivedData);
    Serial.print("Uploading data to ThingSpeak: ");
    Serial.println(url);

    // Make HTTP request
    http.begin(client, url); // Specify the client and URL
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      if (httpResponseCode == 200) {
        Serial.println("Data uploaded successfully!");
      } else {
        Serial.println("Failed to upload data. Check API key and network.");
      }
    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(http.errorToString(httpResponseCode).c_str());
    }

    http.end(); // Free resources
  } else {
    Serial.println("WiFi not connected. Reconnecting...");
    WiFi.begin(ssid, password);
   }

   delay(15000); // Send data every 15 seconds
    
   }
   


}
