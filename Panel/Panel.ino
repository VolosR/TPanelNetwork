#define LV_ATTRIBUTE_TICK_INC IRAM_ATTR
#define TOUCH_MODULES_CST_MUTUAL

#include "esp_log.h"
#include <WiFi.h>
#include "lvgl.h"
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "TouchLib.h"
#include "ui.h"
#include "esp_attr.h"
#include <ESPping.h>

String ssid = "IGK20";
String password = "18072019";


static bool Touch_Int_Flag = false;
int deb = 0;
int ipNum[4]={8,8,8,8};
String targetString="8.8.8.8";

TouchLib touch(Wire, TOUCH_SDA, TOUCH_SCL, CST3240_ADDRESS);

Arduino_DataBus *bus = new Arduino_XL9535SWSPI(IIC_SDA /* SDA */, IIC_SCL /* SCL */, -1 /* XL PWD */,
                                               XL95X5_CS /* XL CS */, XL95X5_SCLK /* XL SCK */, XL95X5_MOSI /* XL MOSI */);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  -1 /* DE */, LCD_VSYNC /* VSYNC */, LCD_HSYNC /* HSYNC */, LCD_PCLK /* PCLK */,
  LCD_B0 /* B0 */, LCD_B1 /* B1 */, LCD_B2 /* B2 */, LCD_B3 /* B3 */, LCD_B4 /* B4 */,
  LCD_G0 /* G0 */, LCD_G1 /* G1 */, LCD_G2 /* G2 */, LCD_G3 /* G3 */, LCD_G4 /* G4 */, LCD_G5 /* G5 */,
  LCD_R0 /* R0 */, LCD_R1 /* R1 */, LCD_R2 /* R2 */, LCD_R3 /* R3 */, LCD_R4 /* R4 */,
  1 /* hsync_polarity */, 20 /* hsync_front_porch */, 2 /* hsync_pulse_width */, 0 /* hsync_back_porch */,
  1 /* vsync_polarity */, 30 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 1 /* vsync_back_porch */,
  10 /* pclk_active_neg */, 6000000L /* prefer_speed */, false /* useBigEndian */,
  0 /* de_idle_high*/, 0 /* pclk_idle_high */);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  LCD_WIDTH /* width */, LCD_HEIGHT /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
  bus, -1 /* RST */, st7701_type9_init_operations, sizeof(st7701_type9_init_operations));

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {

  if(deb==0){
  if (Touch_Int_Flag == true) {
    Touch_Int_Flag = false;
    deb = 1;
    delay(20);
    touch.read();
    if (touch.getPointNum() > 0) {
      data->state = LV_INDEV_STATE_PR;
      TP_Point t = touch.getPoint(0);
      

      /*Set the coordinates*/
      data->point.x = t.x;
      data->point.y = t.y;
      Serial.print("Data x ");
    }
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}else{data->state = LV_INDEV_STATE_REL;}
}

void lvgl_initialization(void)
{
    lv_init();

    uint32_t screen_width = gfx->width();
    uint32_t screen_height = gfx->height();

    // 使用PSRAM分配内存
    lv_color_t *buf_1 = (lv_color_t *)heap_caps_malloc(48 * 1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_color_t *buf_2 = NULL;
    lv_disp_draw_buf_init(&draw_buf, buf_1, NULL, 2 * 480 * 10);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screen_width;
    disp_drv.ver_res = screen_height;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.full_refresh = 1; // 双缓冲全像素刷新
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
}




void networkData()
{ 
  _ui_label_set_property(ui_ssidLbl, _UI_LABEL_PROPERTY_TEXT, String(WiFi.SSID()).c_str());
  _ui_label_set_property(ui_ipLbl, _UI_LABEL_PROPERTY_TEXT, WiFi.localIP().toString().c_str());

  if(WiFi.status()==3)
  _ui_label_set_property(ui_statusLbl, _UI_LABEL_PROPERTY_TEXT, "CONNECTED!");
  else
  _ui_label_set_property(ui_statusLbl, _UI_LABEL_PROPERTY_TEXT, "DISCONNECTED");

   _ui_label_set_property(ui_largeRSSI, _UI_LABEL_PROPERTY_TEXT, String(WiFi.RSSI()).c_str());
  _ui_label_set_property(ui_macLbl, _UI_LABEL_PROPERTY_TEXT, String(WiFi.macAddress()).c_str());
  _ui_label_set_property(ui_subnetLbl, _UI_LABEL_PROPERTY_TEXT, WiFi.subnetMask().toString().c_str());
  _ui_label_set_property(ui_gatewayLbl, _UI_LABEL_PROPERTY_TEXT, WiFi.gatewayIP().toString().c_str());
}

void setup() {

  esp_log_level_set("*", ESP_LOG_DEBUG); // Set all modules to DEBUG level
    esp_log_level_set("wifi", ESP_LOG_VERBOSE);


  Serial.begin(115200);
  Serial.println("lets GO");

  analogWrite(LCD_BL, 120);

  pinMode(TOUCH_INT, INPUT_PULLUP);
  digitalWrite(TOUCH_INT, HIGH);
  

  attachInterrupt(
    TOUCH_INT,
    [] {
      if(deb==0)
      Touch_Int_Flag = true;
      //Serial.println("get_int");
    },
    FALLING);  // Triggered every 1ms
  
 

  Wire.begin(IIC_SDA, IIC_SCL);


//CONNECT TO WIFI
  WiFi.mode(WIFI_MODE_STA); 
  Serial.println("Connecting to WIFI");
  WiFi.begin(ssid, password);  
   while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("CONNECTED :)");

//DISPLAY DRIVER
  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->XL_digitalWrite(TOUCH_RST, LOW);
  delay(200);
  gfx->XL_digitalWrite(TOUCH_RST, HIGH);
  delay(200);

  touch.init();

  lvgl_initialization();
  ui_init();

   
 
   // checkPSRAM();
 networkData();
}

int n = 0;
unsigned long timepassed=0;


void pingFunc(lv_event_t * e)
{
   const IPAddress remote_ip(ipNum[0],ipNum[1],ipNum[2],ipNum[3]);
  Serial.print(remote_ip);
  if (Ping.ping(remote_ip) > 0){
    Serial.printf(" response time : %d/%.2f/%d ms\n", Ping.minTime(), Ping.averageTime(), Ping.maxTime());
      _ui_label_set_property(ui_responseLbl, _UI_LABEL_PROPERTY_TEXT, String(Ping.averageTime()).c_str());
  } else {
    Serial.println(" Error !");
      _ui_label_set_property(ui_responseLbl, _UI_LABEL_PROPERTY_TEXT, "ERROR");
  }
  
}

void refreshNetwork(lv_event_t * e)
{networkData();}

void prepareNumbers(lv_event_t * e)
{}

void setNumbers(lv_event_t * e)
{
   const char *text1 = lv_textarea_get_text(ui_TextArea1);
   const char *text2 = lv_textarea_get_text(ui_TextArea2);
   const char *text3 = lv_textarea_get_text(ui_TextArea3);
   const char *text4 = lv_textarea_get_text(ui_TextArea4);

    // Convert the text to an integer
    ipNum[0] = atoi(text1);
     ipNum[1] = atoi(text2);
      ipNum[2] = atoi(text3);
       ipNum[3] = atoi(text4);
       targetString=String(ipNum[0])+"."+String(ipNum[1])+"."+String(ipNum[2])+"."+String(ipNum[3]);
       _ui_label_set_property(ui_targetLbl, _UI_LABEL_PROPERTY_TEXT, targetString.c_str());
}

void loop() {



  if(deb==1)
  {
    timepassed=millis();
    deb=3;
  }

  if(deb==3 && millis()>timepassed+320)
  {
    deb=0;
  }
  
    lv_task_handler(); /* let the GUI do its work */
    delay(10);
}

void checkPSRAM()
{
    Serial.print(F("Total heap  ")); Serial.println(ESP.getHeapSize());
    Serial.print(F("Free heap   ")); Serial.println(ESP.getFreeHeap());
    Serial.print(F("Total psram ")); Serial.println(ESP.getPsramSize());
    Serial.print(F("Free psram  ")); Serial.println(ESP.getFreePsram());
    Serial.println(F(" "));
}