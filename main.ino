#include <math.h>
#include <string.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <esp_now.h>
#include <FirebaseESP32.h>

#define FIREBASE_HOST "https://test-gps-c8258-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "h0GdUK4AlCzoLgSdHFwU2BB8UQ2x3RKrrUC1cO4h"
#define WIFI_SSID "Hiuhiu"
#define WIFI_PASSWORD "11112222"
FirebaseData firebaseData;

TaskHandle_t Task1Handler;
void Task1_Sensor (void *p);   //đọc value từ MQ2 && Lửa
TaskHandle_t Task2Handler;
void Task2_lcd (void *p);   //
TaskHandle_t Task3Handler;
void Task3_firebase (void *p);
TaskHandle_t Task4Handler;
void Task4_led_buzzer(void *p);

LiquidCrystal_I2C lcd(0x27, 16, 2); 

static double haversine(double lat1, double lon1,
						double lat2, double lon2)
	{
		// distance between latitudes
		// and longitudes
		double dLat = (lat2 - lat1) *
					M_PI / 180.0;
		double dLon = (lon2 - lon1) *
					M_PI / 180.0;

		// convert to radians
		lat1 = (lat1) * M_PI / 180.0;
		lat2 = (lat2) * M_PI / 180.0;

		// apply formulae
		double a = pow(sin(dLat / 2), 2) +
				pow(sin(dLon / 2), 2) *
				cos(lat1) * cos(lat2);
		double rad = 6371;
		double c = 2 * asin(sqrt(a));
		return rad * c;
	}

double from_UTE;
double from_HCMUT;
String chuoi;
void take_FB_data_cacu_shortest_des(){
  double val1, val2;
  String chuoi1, chuoi2;
  byte moc;
   if (Firebase.getString(firebaseData, "/User_using/User_0/GPS_0"))
  {
    chuoi=firebaseData.stringData();
    while(chuoi=="0"){
      Firebase.getString(firebaseData, "/User_using/User_0/GPS_0");
      chuoi=firebaseData.stringData();
      delay(500);
      }
    for (int i = 0; i < chuoi.length(); i++) {
        if (chuoi.charAt(i) == ',') {
            moc = i; //Tìm vị trí của dấu ","
        }
    }
    chuoi1 = chuoi;
    chuoi2 = chuoi;
    chuoi1.remove(moc); //Tách giá trị thanh trượt 1 ra chuoi1
    chuoi2.remove(0, moc + 1); ////Tách giá trị thanh trượt 2 ra chuoi2
    val1 = chuoi1.toDouble(); //Chuyển chuoi1 thành số
    val2 = chuoi2.toDouble(); //Chuyển chuoi2 thành số
    //In ra serial
    Serial.print(val1,7);
    Serial.print(" and ");
    Serial.println(val2,7);
     from_UTE= haversine(val1,val2,10.8507214,106.7697336); // UTE
     from_HCMUT= haversine(val1,val2,10.8805587,106.803188); // HCMUT
    Serial.print("from_UTE: ");
    Serial.print(from_UTE,7);
    Serial.print(" from_HCMUT: ");
    Serial.println(from_HCMUT,7);
  }
}

typedef struct struct_fbdb{
    char des_User_MQ2[100];
    char des_User_Fire[100];
    char des_User_Warning[100];
    char des_Rom_Warning[100];
    char User_MQ2[100];
    char User_Fire[100];
    char User_Warning[100];
    char GPS_des [100];
}struct_fbdb;
struct_fbdb str;

    char MQ2_str[]= "/MQ2_value/";
    char Fire_str[] = "/Fire_value/";
    char Waring_str []= "/Warning/";
    char User_using[]= "/User_using";
    char UTE_des[]="/From_UTE";
    char HCMUT_des[]="/From_HCMUT";
    char User[]= "/User_0";
    char GPS_str[]="/GPS_0/";
    char Rom_str[]="/ROOM/";
void control_data_to_fb(char s[]){
        strcpy(str.des_User_MQ2,s);
        strcat(str.des_User_MQ2,User);
        strcat(str.des_User_MQ2,MQ2_str);
        
        strcpy(str.des_User_Fire,s);
        strcat(str.des_User_Fire,User);
        strcat(str.des_User_Fire,Fire_str);
        
        strcpy(str.des_User_Warning,s);
        strcat(str.des_User_Warning,User);
        strcat(str.des_User_Warning,Waring_str);

        strcpy(str.GPS_des,s);
        strcat(str.GPS_des,User);
        strcat(str.GPS_des,GPS_str);

        strcpy(str.des_Rom_Warning,s);
        strcat(str.des_Rom_Warning,User);
        strcat(str.des_Rom_Warning,Rom_str);
        
        strcpy(str.User_MQ2,User_using);
        strcat(str.User_MQ2,User);
        strcat(str.User_MQ2,MQ2_str);
        
        strcpy(str.User_Fire,User_using);
        strcat(str.User_Fire,User);
        strcat(str.User_Fire,Fire_str);
        
        strcpy(str.User_Warning,User_using);
        strcat(str.User_Warning,User);
        strcat(str.User_Warning,Waring_str);
}

// Must match the sender structure
typedef struct struct_message {
    int MQ2_data;
    int fire_data;
} struct_message;
// Create a struct_message called myData
struct_message myData;
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  Serial.print("MQ2 value rc from salve 1: ");
  Serial.println(myData.MQ2_data);
  Serial.print("fire value rc from salve 1: ");
  Serial.println(myData.fire_data);
  Serial.println(); 
}

void connect_wifi()
{
       Serial.println("Connecting to ");
       Serial.println(WIFI_SSID);
       WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) 
     {
            delay(500);
            Serial.print(".");
     }
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.print("Wi-Fi Channel: ");
      Serial.println(WiFi.channel());
  }

int so_chuSo(int value)
{
    int i=1;
    while(value/10!=0)
    {
      value=value/10;
      i++;
    }
    return i;
}

uint8_t MQ2=34;
uint8_t fire=35;
uint8_t led_red= 4;
uint8_t led_green= 15;
uint8_t buzzer = 33;

void setup()
{
    myData.fire_data=1;
    myData.MQ2_data=300;
	Serial.begin(115200);

    pinMode(MQ2,INPUT);
    pinMode(fire,INPUT);
    pinMode(led_red,OUTPUT);
    pinMode(led_green,OUTPUT);
    pinMode(buzzer,OUTPUT);

    lcd.init();
    lcd.backlight();

     connect_wifi();

     WiFi.mode(WIFI_AP_STA);

     Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
     Firebase.reconnectWiFi(true);

     take_FB_data_cacu_shortest_des();
     if(from_UTE>from_HCMUT)
    {
     control_data_to_fb(HCMUT_des);
      Firebase.deleteNode(firebaseData,"/From_UTE/User_0");
     }
     else
    {
      control_data_to_fb(UTE_des);
      Firebase.deleteNode(firebaseData,"/From_HCMUT/User_0");
     }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

    xTaskCreate(Task1_Sensor,"Task1",4096,NULL,3,&Task1Handler);
    xTaskCreate(Task2_lcd,"Task2",4096,NULL,1,&Task2Handler);
    xTaskCreate(Task3_firebase,"Task3",4*4096,NULL,2,&Task3Handler);
    xTaskCreate(Task4_led_buzzer,"Task4",4096,NULL,1,&Task4Handler);
    
}

int HARD_WARNING,SMOKE_WARNING,FIRE_WARNING;
int MQ2_value;
int fire_value;
int Max_MQ2_value;
int Rom;
void Task1_Sensor (void *p)
{
    while (1)
    {
        int have_smoke,have_fire;
        int value = analogRead(MQ2);
         MQ2_value= map(value,0,4500,0,2000);; 
         fire_value= digitalRead(fire);
         Serial.print("MQ2 value= ");
         Serial.println(MQ2_value);
         Serial.print("fire = ");
         Serial.println(fire_value);
         Serial.println(); 

        if(MQ2_value>myData.MQ2_data)
         Max_MQ2_value=MQ2_value;
         else
         Max_MQ2_value=myData.MQ2_data;

        if(MQ2_value>1200 || myData.MQ2_data>1200)
            have_smoke=1;
        else
            have_smoke=0;

        if(fire_value==0 || myData.fire_data==0)
            have_fire=1;
        else
            have_fire=0;
        
        if (have_smoke && have_fire)
            HARD_WARNING=1;
        else
            HARD_WARNING=0;

        if(HARD_WARNING==1){
            if(fire_value==0 && MQ2_value>1200)
                Rom=1;
            else if(myData.fire_data==0 && myData.MQ2_data>1200)
                Rom=2;
        }

        if (have_smoke == 1)
            SMOKE_WARNING=1;
        else
            SMOKE_WARNING=0;      
        
        if (have_fire ==1)
            FIRE_WARNING=1;
        else
            FIRE_WARNING=0;
        vTaskDelay(100);
    }
    
}

void Task2_lcd (void *p)
{
    while (1)
    {
        if(SMOKE_WARNING==0 && FIRE_WARNING ==0 && HARD_WARNING==0)
        {
            int so_chuso=so_chuSo(Max_MQ2_value);
            lcd.setCursor(0,0);
            lcd.print("Smoke = ");
            lcd.setCursor(8,0);
            lcd.print(Max_MQ2_value);
            lcd.setCursor(8+so_chuso,0);
            lcd.print(" ");
            lcd.setCursor(0,1);
            lcd.print("     NORMAL     ");

        }

        if (SMOKE_WARNING==1 && FIRE_WARNING==0)
        {           
            lcd.setCursor(0,0);
            lcd.print("     WARNING     ");
            lcd.setCursor(0,1);
            lcd.print("    SMOKE||GAS   ");
        }
 
        
        if (FIRE_WARNING==1 && SMOKE_WARNING==0)
        {           
            lcd.setCursor(0,0);
            lcd.print("     WARNING     ");
            lcd.setCursor(0,1);
            lcd.print("    HAVE FIRE    ");
        }

        
        if (HARD_WARNING==1)
        {
            lcd.setCursor(0,0);
            lcd.print("WARNING WARNING   ");
            lcd.setCursor(0,1);
            lcd.print(" FIRE FIRE FIRE   ");
        }
        vTaskDelay(500);
    }
    
}

void Task3_firebase (void *p)
{
    while (1)
    {
        Firebase.setInt(firebaseData,str.des_User_MQ2,Max_MQ2_value);
        Firebase.setInt(firebaseData,str.des_User_Fire,FIRE_WARNING);
        Firebase.setInt(firebaseData,str.des_User_Warning,HARD_WARNING);
        Firebase.setString(firebaseData,str.GPS_des,chuoi);
        
        if(HARD_WARNING)
            Firebase.setInt(firebaseData,str.des_Rom_Warning,Rom);
        else
            Firebase.deleteNode(firebaseData,str.des_Rom_Warning);
            
        //Firebase.setInt(firebaseData,str.User_MQ2,Max_MQ2_value);
       // Firebase.setInt(firebaseData,str.User_Fire,FIRE_WARNING);
       // Firebase.setInt(firebaseData,str.User_Warning,HARD_WARNING);

        vTaskDelay(500);
    }
    
}

void Task4_led_buzzer(void *p)
{
    uint8_t u=1;
    while (1)
    {
        
        if(SMOKE_WARNING || FIRE_WARNING || HARD_WARNING)
            {
                
                digitalWrite(led_green,0);
                digitalWrite(led_red,u);
                analogWrite(buzzer,50);
                u=!u;
            }
        else
            {
                u=1;
                digitalWrite(led_green,1);
                digitalWrite(led_red,0);
                analogWrite(buzzer,0);
            }
            vTaskDelay(500);
    }
    
}

void loop(){}

