#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include "ArduiKalman.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>  

#define I2C_SDA 21
#define I2C_SCL 22
#define DHTPIN 19        
#define DHTTYPE DHT11    
#define WATER_SENSOR_PIN 23

int stateNum = 1;
int measureNum = 1;

float xc[1];              
float xp[1];              
float xc_humidity[1];     
float xp_humidity[1];     
float xc_pressure[1];     
float xp_pressure[1];     

float A[1][1];            
float Q[1][1];            
float R[1][1];            
float H[1][1];            
float P[1][1];            

const char* ssid = "01";       
const char* password = "12345678"; 

const char* apiKey = "f8487154b1b7ab8c5c3370486bac8994";
const char* weatherApiUrl = "http://api.openweathermap.org/data/2.5/weather?q=Zagazig&appid=f8487154b1b7ab8c5c3370486bac8994&units=metric";

const String firebaseURL = "https://sssss-c671d-default-rtdb.firebaseio.com/";
const String firebaseAPIKey = "AIzaSyD7FfiPS5QZq5YBpf8dkDbU4MxXmL_3bJA";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 3600000);

float precipitation = 0.0; 
float windSpeed = 0.0;
float pressure = 0.0;    

KalmanFilter m_kf_temp;   
KalmanFilter m_kf_humidity;  
KalmanFilter m_kf_pressure; 

DHT dht(DHTPIN, DHTTYPE);    
Adafruit_BMP280 bmp;         

#define NUM_CLASSES 5
#define NUM_FEATURES 4

float class_priors[NUM_CLASSES] = {0.03835616, 0.06392694, 0.44383562, 0.01826484, 0.43561644};

float class_means[NUM_CLASSES][NUM_FEATURES] = {
    {-0.45068983,  0.04142778, -0.06754036, -0.61509104},
    {-0.45068983,  0.03711242, -0.07550493, -0.53450661},
    { 0.51565673, -0.38821937, -0.10207665,  0.28383328},
    { 0.74235682, -1.41622884, -1.49085661,  0.97113293},
    {-0.45068983,  0.44583093,  0.18353967, -0.1973089}
};

float class_variances[NUM_CLASSES][NUM_FEATURES] = {
    {1.00000000e-09, 1.42941586e+00, 1.40520875e+00, 4.03585034e-01},
    {1.00000000e-09, 8.52184130e-01, 1.01616286e+00, 7.53459168e-01},
    {1.67568937e+00, 4.60202201e-01, 6.14308081e-01, 1.23887816e+00},
    {1.03631662e+00, 1.49563198e-01, 1.84361252e-01, 7.61950943e-01},
    {1.00000000e-09, 1.13274923e+00, 1.25038280e+00, 6.19496299e-01}
};

float calculateProbability(float x, float mean, float variance) {
    float exponent = exp(-pow(x - mean, 2) / (2 * variance));
    return (1.0 / sqrt(2 * M_PI * variance)) * exponent;
}

int predictClass(float features[NUM_FEATURES]) {
    float probabilities[NUM_CLASSES];
    for (int c = 0; c < NUM_CLASSES; c++) {
        probabilities[c] = log(class_priors[c]); 
        for (int f = 0; f < NUM_FEATURES; f++) {
            probabilities[c] += log(calculateProbability(features[f], class_means[c][f], class_variances[c][f]));
        }
    }

    int predicted_class = 0;
    float max_probability = probabilities[0];
    for (int c = 1; c < NUM_CLASSES; c++) {
        if (probabilities[c] > max_probability) {
            max_probability = probabilities[c];
            predicted_class = c;
        }
    }
    return predicted_class;
}

String getWeatherLabel(int class_index) {
    switch (class_index) {
        case 0: return "Drizzle";
        case 1: return "Fog";
        case 2: return "Rain";
        case 3: return "Snow";
        case 4: return "Sun";
        default: return "Unknown";
    }
}

void fetchWeatherData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(weatherApiUrl);

    int httpResponseCode = http.GET();
    if (httpResponseCode == 200) {
      String responseBody = http.getString();

      int precipitationIndex = responseBody.indexOf("\"rain\"");
      if (precipitationIndex != -1) {
        int rainValueStart = responseBody.indexOf("1h", precipitationIndex) + 4;
        int rainValueEnd = responseBody.indexOf(",", rainValueStart);
        precipitation = responseBody.substring(rainValueStart, rainValueEnd).toFloat();
      } else {
        precipitation = 0.0;
      }

      int pressureIndex = responseBody.indexOf("\"pressure\"");
      if (pressureIndex != -1) {
        int pressureValueStart = responseBody.indexOf(":", pressureIndex) + 1;
        int pressureValueEnd = responseBody.indexOf(",", pressureValueStart);
        pressure = responseBody.substring(pressureValueStart, pressureValueEnd).toFloat();
      } else {
        pressure = 0.0;
      }

      int windSpeedIndex = responseBody.indexOf("\"speed\"");
      if (windSpeedIndex != -1) {
        int windValueStart = responseBody.indexOf(":", windSpeedIndex) + 1;
        int windValueEnd = responseBody.indexOf(",", windValueStart);
        windSpeed = responseBody.substring(windValueStart, windValueEnd).toFloat();
      }
    } else {
      Serial.print("Error in API request. HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi not connected. Cannot fetch weather data.");
  }
}

String getCurrentTimestamp() {
  timeClient.update();
  unsigned long timestamp = timeClient.getEpochTime();
  timestamp += 2 * 3600;
  String daysOfWeek[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  String currentDay = daysOfWeek[weekday(timestamp) - 1];
  String currentDate = currentDay + " " + String(day(timestamp)) + "/" + String(month(timestamp)) + "/" + String(year(timestamp)) +
                       " " + String(hour(timestamp)) + ":" + String(minute(timestamp)) + ":" + String(second(timestamp));
  return currentDate;
}

void sendToFirebase(float temperature, float temp_max, float temp_min, float pressure, float humidity, bool waterDetected, float precipitation, float windSpeed, int predicted_class) {
  Serial.println("Sending data to Firebase:");

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String timestamp = getCurrentTimestamp();
    String temperaturePath = firebaseURL + "sensors/temperature/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String temperaturePayload = String(temperature);

    http.begin(temperaturePath);
    http.addHeader("Content-Type", "application/json");
    int temperatureResponse = http.PUT(temperaturePayload);
    if (temperatureResponse > 0) {
      Serial.println("Temperature data sent successfully.");
    } else {
      Serial.println("Failed to send temperature data.");
    }
    http.end();

    String temp_maxPath = firebaseURL + "sensors/temp_max/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String temp_maxPayload = String(temp_max);

    http.begin(temp_maxPath);
    http.addHeader("Content-Type", "application/json");
    int temp_maxResponse = http.PUT(temp_maxPayload);
    if (temp_maxResponse > 0) {
      Serial.println("temp_max data sent successfully.");
    } else {
      Serial.println("Failed to send temperature data.");
    }
    http.end();

    String temp_minPath = firebaseURL + "sensors/temp_min/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String temp_minPayload = String(temp_min);

    http.begin(temp_minPath);
    http.addHeader("Content-Type", "application/json");
    int temp_minResponse = http.PUT(temp_minPayload);
    if (temp_minResponse > 0) {
      Serial.println("temp_min data sent successfully.");
    } else {
      Serial.println("Failed to send temperature data.");
    }
    http.end();

    String pressurePath = firebaseURL + "sensors/pressure/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String pressurePayload = String(pressure);

    http.begin(pressurePath);
    http.addHeader("Content-Type", "application/json");
    int pressureResponse = http.PUT(pressurePayload);
    if (pressureResponse > 0) {
      Serial.println("pressure data sent successfully.");
    } else {
      Serial.println("Failed to send pressure data.");
    }
    http.end();

    String humidityPath = firebaseURL + "sensors/humidity/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String humidityPayload = String(humidity);

    http.begin(humidityPath);
    http.addHeader("Content-Type", "application/json");
    int humidityResponse = http.PUT(humidityPayload);
    if (humidityResponse > 0) {
      Serial.println("humidity data sent successfully.");
    } else {
      Serial.println("Failed to send humidity data.");
    }
    http.end();
    String waterDetectedPath = firebaseURL + "sensors/waterDetected/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String waterDetectedPayload = String(waterDetected);

    http.begin(waterDetectedPath);
    http.addHeader("Content-Type", "application/json");
    int waterDetectedResponse = http.PUT(waterDetectedPayload);
    if (waterDetectedResponse > 0) {
      Serial.println("waterDetected data sent successfully.");
    } else {
      Serial.println("Failed to send waterDetected data.");
    }
    http.end();
    String precipitation = firebaseURL + "sensors/precipitation/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String precipitationPayload = String(precipitation);

    http.begin(precipitation);
    http.addHeader("Content-Type", "application/json");
    int precipitationResponse = http.PUT(precipitationPayload);
    if (precipitationResponse > 0) {
      Serial.println("Water Level data sent successfully.");
    } else {
      Serial.println("Failed to send Water Level data.");
    }
    http.end();
    String windSpeedPath = firebaseURL + "sensors/windSpeed/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String windSpeedPayload = String(windSpeed);

    http.begin(windSpeedPath);
    http.addHeader("Content-Type", "application/json");
    int windSpeedResponse = http.PUT(windSpeedPayload);
    if (windSpeedResponse > 0) {
      Serial.println("Water Level data sent successfully.");
    } else {
      Serial.println("Failed to send Water Level data.");
    }
    http.end();

    String predicted_classPath = firebaseURL + "sensors/predicted_class/" + timestamp + ".json?auth=" + firebaseAPIKey;
    String predicted_classPayload = String(predicted_class);

    http.begin(predicted_classPath);
    http.addHeader("Content-Type", "application/json");
    int predicted_classResponse = http.PUT(predicted_classPayload);
    if (predicted_classResponse > 0) {
      Serial.println("predicted_class data sent successfully.");
    } else {
      Serial.println("Failed to send predicted_class data.");
    }
    http.end();

  } else {
    Serial.println("WiFi disconnected. Cannot send data.");
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(WATER_SENSOR_PIN, INPUT);

  WiFi.begin(ssid, password);

  m_kf_temp.init(stateNum, measureNum, &A[0][0], &P[0][0], &Q[0][0], &H[0][0], &R[0][0], &xp[0], &xc[0]);
  m_kf_humidity.init(stateNum, measureNum, &A[0][0], &P[0][0], &Q[0][0], &H[0][0], &R[0][0], &xp_humidity[0], &xc_humidity[0]);
  m_kf_pressure.init(stateNum, measureNum, &A[0][0], &P[0][0], &Q[0][0], &H[0][0], &R[0][0], &xp_pressure[0], &xc_pressure[0]);

  A[0][0] = 1.0f;  
  H[0][0] = 1.0f;  
  Q[0][0] = 0.01f; 
  R[0][0] = 10.0f; 
  P[0][0] = 1.0f;  

  xc[0] = dht.readTemperature();
  xc_humidity[0] = dht.readHumidity();
  xc_pressure[0] = bmp.readPressure() / 100.0F; 

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  timeClient.begin();
}

void loop() {
  fetchWeatherData();

  float dhtTemperature = dht.readTemperature();
  float dhtHumidity = dht.readHumidity();
  float bmpPressure = bmp.readPressure() / 100.0F;

  float *predict_temp = m_kf_temp.predict();
  float measurement_temp[measureNum] = {dhtTemperature};
  float *correct_temp = m_kf_temp.correct(measurement_temp);
  float estimated_temp = correct_temp[0];

  float *predict_humidity = m_kf_humidity.predict();
  float measurement_humidity[measureNum] = {dhtHumidity};
  float *correct_humidity = m_kf_humidity.correct(measurement_humidity);
  float estimated_humidity = correct_humidity[0];

  float *predict_pressure = m_kf_pressure.predict();
  float measurement_pressure[measureNum] = {bmpPressure};
  float *correct_pressure = m_kf_pressure.correct(measurement_pressure);
  float estimated_pressure = correct_pressure[0];

  bool waterDetected = digitalRead(WATER_SENSOR_PIN);

  static float max_temp = estimated_temp;  
  static float min_temp = estimated_temp;  

  if (estimated_temp > max_temp) {
      max_temp = estimated_temp;
  }
  if (estimated_temp < min_temp) {
      min_temp = estimated_temp;
  }

  float features[NUM_FEATURES] = {
      precipitation,  
      3.0,       
      3.0,       
      3.54       
  };

  int predicted_class = predictClass(features);
  String predicted_weather = getWeatherLabel(predicted_class);

  Serial.print("Temp Max (Filtered): "); Serial.print(max_temp);
  Serial.print(", Temp Min (Filtered): "); Serial.print(min_temp);
  Serial.print(", Humidity (Unfiltered): "); Serial.print(dhtHumidity);
  Serial.print(", Humidity (Filtered): "); Serial.print(estimated_humidity);
  Serial.print(", Pressure (Unfiltered): "); Serial.print(pressure);
  Serial.print(", Pressure (Filtered): "); Serial.print(pressure);
  Serial.print(", Water Detected: "); Serial.print(waterDetected ? "Yes" : "No");
  Serial.print(", Precipitation: "); Serial.print(precipitation);
  Serial.print(", Wind Speed: "); Serial.print(windSpeed);
  Serial.println(" Predicted Weather Type: " + predicted_weather); 

  sendToFirebase(estimated_temp, max_temp, min_temp, pressure, estimated_humidity, waterDetected, precipitation, windSpeed, predicted_class);
  delay(1000);
}