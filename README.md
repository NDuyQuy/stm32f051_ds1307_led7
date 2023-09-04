# stm32f051_ds1307_led7

# Description
	Using STM32F051 comunication with real-time module DS1307 and display on 4 digits led 7 segment . 
	The comunication between STM32F051 and DS1307 is using I2C protocol(by software).

# pin connection

## STM32F051 DISCOVERY KIT                            Led 7 Segment
    PORT_C PIN0->PIN6            						LED A -> G
	PORT A PIN9->PIN12									LEDPOWER 1 -> 4
## STM32F051 DISCOVERY KIT                            DS1307
	PORT B PIN6											DS1307_SCL
	PORT B PIN7											DS1307_SDA
## STM32F051 DISCOVERY KIT                            Button
	PORT A PIN0											BUTTON1(SETTING TIME)
	PORT A PIN2											BUTTON2(UP TIME)
	PORT A PIN3											BUTTON3(DOWN TIME)