# Smart-weather-monitoring-system-using-freertos-version-3.0
Smart Weather Monitoring System V 3.0 - FREE RTOS - Multi thread model
Task 1: Every 1 second, read date & time from DS1307 RTC and write into the LCD Main menu.

void Task1_ReadRTC_WriteLCD_1Sec(void);

 

Task 2: Every 5 seconds, read the temperature from the LM35 temperature sensor and write it to the global buffer lm35_data, and set the lm35_flag.

void Task2_ReadLM35_WriteLCD_5Sec(void);

 

Task 3: Check internet status continuously, whenever the internet is not working then clear internet_status flag and write temperature along with timestamp into EEPROM.

Otherwise set internet_status flag.

Void Task3_Check_InternetStatus(void);

 

Task 4: Whenever both LM35_flag and internet_status flags set, read the temperature value from the lm35_data global buffer and send it to kernel Masters webserver using ESP8266 Wi-Fi Module and reset the lm35_flag and internet_status flag.

Void Task4_Wi-Fi_Tx(void);

 

Task 5: SWMS Configuration:  Whenever ENTER switch presses, invoke the Task5 function. Using the Task5 function, the user can configure the date and time.

 void Task5_SWMS_Config(void);

 

LCD output – Configuration Menu:

H H : M M : S S

D D / M M / Y Y


LCD output – Main Menu:

H H : M M : S S   T : x x   D C 

D D / M M / Y Y       O K

 
x x -> Temperature value

D -> Degree symbol

C -> Centigrade

Ok -> Internet status (Network is connected)

Error -> (Network is not Connected)
