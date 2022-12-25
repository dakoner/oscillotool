#include <driver/rmt.h>
#include <Arduino.h>
gpio_num_t led0_pin = GPIO_NUM_22; // LED

// setting PWM properties
const int led0Channel = 0;
const int led0Resolution = 4;

gpio_num_t led1_pin = GPIO_NUM_23; // LED

// setting PWM properties
const int led1Channel = 1;
const int led1Resolution = 4;


hw_timer_t * timer = NULL;

// volatile uint32_t isrTriggerCounter = 0;
// volatile uint32_t lastTriggerIsrAt = 0;
// volatile SemaphoreHandle_t triggerSemaphore;
// portMUX_TYPE triggerMux = portMUX_INITIALIZER_UNLOCKED;


volatile uint32_t isrTimerCounter = 0;
volatile uint32_t lastTimerIsrAt = 0;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrTimerCounter++;
  lastTimerIsrAt = micros();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  digitalWrite(led0_pin, HIGH);
  digitalWrite(led0_pin, LOW);
}

// void ARDUINO_ISR_ATTR camera_trigger_isr() {
//     portENTER_CRITICAL_ISR(&triggerMux);
//     isrTriggerCounter++;
//     lastTriggerIsrAt = micros();
//     portEXIT_CRITICAL_ISR(&triggerMux);
//     // Give a semaphore that we can check in the loop
//     xSemaphoreGiveFromISR(triggerSemaphore, NULL);
//     //digitalWrite(led_pin, HIGH);
//     //delayMicroseconds(1000);
//     digitalWrite(led_pin, HIGH);
//     //digitalWrite(camera_trigger_pin, LOW);
//     timerAlarmEnable(timer);
// }



void enable_pwm(double freq, int duty)
{
    int freq_out = ledcSetup(led0Channel, freq, led0Resolution);
    Serial.print("Freq out: ");
    Serial.print(freq_out);
    Serial.println();
    ledcAttachPin(led0_pin, led0Channel);
    ledcWrite(led0Channel, duty);
}

void disable_pwm()
{
    ledcDetachPin(led0_pin);
}



// void handleTrigger() {

//     if (xSemaphoreTake(triggerSemaphore, 0) == pdTRUE){
//         uint32_t isrCount = 0, isrTime = 0;
//         // Read the interrupt count and time
//         portENTER_CRITICAL(&triggerMux);
//         isrCount = isrTriggerCounter;
//         isrTime = lastTriggerIsrAt;
//         portEXIT_CRITICAL(&triggerMux);
//         uint32_t dm = micros()-isrTime;
        
//         Serial.print("Camera triggered at ");
//         Serial.print(isrTime);
//         Serial.print(" loop latency ");
//         Serial.print(dm);

//         Serial.print(" total of ");
//         Serial.print(isrCount);
//         Serial.println();
//     } 
// }

// void handleTimer() {

//     if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
//         uint32_t isrCount = 0, isrTime = 0;
//         // Read the interrupt count and time
//         portENTER_CRITICAL(&timerMux);
//         isrCount = isrTimerCounter;
//         isrTime = lastTimerIsrAt;
//         portEXIT_CRITICAL(&timerMux);
//         uint32_t dm = micros()-isrTime;
        
//         Serial.print("Timer triggered at ");
//         Serial.print(isrTime);
//         Serial.print(" loop latency ");
//         Serial.print(dm);

//         Serial.print(" total of ");
//         Serial.print(isrCount);

//         Serial.println();
//     }
// }


void process(String s)
{
    Serial.print("Process: ");
    Serial.println(s);
    if (s.length() > 0)
    {
        char cmd = s[0];
        // Add PWM as alternative to strobe signal
        if (cmd == 'P')
        {
            String arg = s.substring(1);
            int idx = arg.indexOf(' ');
            double freq = arg.substring(0, idx).toDouble();
            int duty = arg.substring(idx).toInt();

            enable_pwm(freq, duty);
            Serial.print("PWM ");
            Serial.print(freq);
            Serial.print(" ");
            Serial.print(duty);
            Serial.println();
        }
        if (cmd == 'S')
        {
            int arg = s.substring(1).toInt();
            disable_pwm();
            digitalWrite(led0_pin, HIGH);
            digitalWrite(led1_pin, HIGH);
            if (arg < 1000)
                delayMicroseconds(arg);
            else
                delay(arg / 1000);
            digitalWrite(led0_pin, LOW);
            digitalWrite(led1_pin, LOW);
            Serial.print("Pulse ");
            Serial.print(arg);
            Serial.println();
        }
        if (cmd == 'T')
        {
            Serial.println("trigger");
            disable_pwm();
            noInterrupts();
            digitalWrite(led0_pin, LOW);
            digitalWrite(led1_pin, LOW);
            delay(1);

            digitalWrite(led0_pin, HIGH);
            delayMicroseconds(100);

            digitalWrite(led1_pin, HIGH);
            delayMicroseconds(100);
            digitalWrite(led1_pin, LOW);
            delayMicroseconds(100);
            digitalWrite(led1_pin, HIGH);
            delayMicroseconds(50);
            digitalWrite(led1_pin, LOW);
            delayMicroseconds(50);
            digitalWrite(led1_pin, HIGH);
            delayMicroseconds(10);
            digitalWrite(led1_pin, LOW);
            delayMicroseconds(10);
            digitalWrite(led1_pin, HIGH);
            delayMicroseconds(1);
            digitalWrite(led1_pin, HIGH);
            delayMicroseconds(1);
            digitalWrite(led1_pin, LOW);

            delayMicroseconds(100);
            digitalWrite(led0_pin, LOW);
            interrupts();
        }
        if (cmd == 'Z') {
                        
            rmt_config_t config;
            rmt_item32_t items[1];
            
            // put your setup code here, to run once:
            config.rmt_mode = RMT_MODE_TX;
            config.channel = RMT_CHANNEL_0;
            config.gpio_num = led1_pin;
            config.mem_block_num = 1;
            config.tx_config.loop_en = 0;
            config.tx_config.carrier_en = 0;
            config.tx_config.idle_output_en = 1;
            config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
            config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
            config.clk_div = 4; // 80MHx / 4 = 20MHz 0r 50nS per count
            
            rmt_config(&config);
            rmt_driver_install(config.channel, 0, 0);  //  rmt_driver_install(rmt_channel_t channel, size_t rx_buf_size, int rmt_intr_num)
            
            items[0].duration0 = 1;
            items[0].level0 = 1;
            items[0].duration1 = 0;
            items[0].level1 = 0;  
            digitalWrite(led0_pin, LOW);
            digitalWrite(led1_pin, LOW);
            delayMicroseconds(1);

            digitalWrite(led0_pin, HIGH);
            delayMicroseconds(1);

            rmt_write_items(config.channel, items, 1, 1);

            delayMicroseconds(1);

            digitalWrite(led0_pin, LOW);
            rmt_driver_uninstall(config.channel); 

        }
        if (cmd == 'L')
        {
            int arg = s.substring(1).toInt();
            disable_pwm();
            if (arg == 0)
            {
                digitalWrite(led0_pin, LOW);
            }
            else if (arg == 1)
            {
                digitalWrite(led0_pin, HIGH);
            }

            Serial.print("Light state ");
            Serial.print(arg);
            Serial.println();
        }
        // if (cmd == 'C')
        // {
        //     int arg = s.substring(1).toInt();
        //     if (arg == 0)
        //     {
        //         digitalWrite(camera_trigger_pin, LOW);
        //     }
        //     else if (arg == 1)
        //     {
        //         digitalWrite(camera_trigger_pin, HIGH);
        //     }

        //     Serial.print("Camera state ");
        //     Serial.print(arg);
        //     Serial.println();
        // }
        // if (cmd == 'Q')
        // {
        //     int arg = s.substring(1).toInt();
        //     digitalWrite(camera_trigger_pin, HIGH);
        //     if (arg < 1000)
        //         delayMicroseconds(arg);
        //     else
        //         delay(arg / 1000);
        //     digitalWrite(camera_trigger_pin, LOW);

        //     Serial.print("Camera strobe ");
        //     Serial.print(arg);
        //     Serial.println();
        // }
        // if (cmd == 'X')
        // {
        //     String arg = s.substring(1);
        //     int idx = arg.indexOf(' ');
        //     int light = arg.substring(0, idx).toInt();
        //     int camera = arg.substring(idx).toInt();
        //     timerAlarmWrite(timer, light, false);
        //     Serial.print(" with light delay set to ");
        //     Serial.println(light);
        //     //disable_pwm();
        //     //digitalWrite(led_pin, LOW); // light off
        //     digitalWrite(camera_trigger_pin, LOW); // camera off
        //     //delayMicroseconds(0);
        //     digitalWrite(camera_trigger_pin, HIGH); // camera on
        //     delayMicroseconds(camera);
        //     // delayMicroseconds(camera);
        //     //digitalWrite(led_pin, HIGH); // light on
        //     // delayMicroseconds(light);
        //     // digitalWrite(led_pin, LOW); // light off
        //     digitalWrite(camera_trigger_pin, LOW); // camera off
        //     //delay(1);

        //     Serial.print("Sync flash and camera ");
        //     Serial.print(light);
        //     Serial.print(" ");
        //     Serial.print(camera);
        //     Serial.print(" ");
        //     Serial.print(micros());
        //     Serial.println();
        // }
    }
}
String line;

void handleSerial() {

    while (Serial.available())
    {
        int c = Serial.read();
        Serial.print((char)c);
        if (c == '\n' || c == '\r')
        {

            process(line);
            line = "";
        }
        else
        {
            line += (char)c;
        }
    }
}



void setup()
{
    Serial.begin(115200);
    // pinMode(camera_trigger_pin, OUTPUT);
    // digitalWrite(camera_trigger_pin, LOW);
    // pinMode(camera_strobe_pin, INPUT_PULLUP);
	// attachInterrupt(camera_strobe_pin, camera_trigger_isr, FALLING);

    pinMode(led0_pin, OUTPUT);
    digitalWrite(led0_pin, LOW);

    pinMode(led1_pin, OUTPUT);
    digitalWrite(led1_pin, LOW);

    // Create semaphore to inform us when the timer has fired
    // triggerSemaphore = xSemaphoreCreateBinary();
    timerSemaphore = xSemaphoreCreateBinary();


    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);

    Serial.println("ready");
}


void loop()
{
    // handleTrigger();
    // handleTimer();
    handleSerial();

}
