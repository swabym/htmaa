// Manar-ul Islam Swaby
// 12/ 2016
// hello.TxRxRGB.44.c
//
// step response transmit-receive hello-world
//    9600 baud FTDI interface
//
// RGB LED software PWM hello-world
//
//
#include <avr/io.h>
#include <util/delay.h>

//rgb.44
#define output(directions,pin) (directions |= pin) // set port direction for output
#define set(port,pin) (port |= pin) // set port pin
#define clear(port,pin) (port &= (~pin)) // clear port pin
#define pin_test(pins,pin) (pins & pin) // test for port pin
#define bit_test(byte,bit) (byte & (1 << bit)) // test for bit set
#define PWM_delay() _delay_us(25) // PWM delay

#define led_port PORTA
#define led_direction DDRA
#define red (1 << PA2)
#define green (1 << PA0)
#define blue (1 << PA1)
//#define button_port PORTA
//#define button (1 << PAO)


//txrx.44
//#define output(directions,pin) (directions |= pin) // set port direction for output
//#define set(port,pin) (port |= pin) // set port pin
//#define clear(port,pin) (port &= (~pin)) // clear port pin
//#define pin_test(pins,pin) (pins & pin) // test for port pin
//#define bit_test(byte,bit) (byte & (1 << bit)) // test for bit set
#define bit_delay_time 102 // bit delay for 9600 with overhead
#define bit_delay() _delay_us(bit_delay_time) // RS232 bit delay
#define half_bit_delay() _delay_us(bit_delay_time/2) // RS232 half bit delay
#define settle_delay() _delay_us(100) // settle delay
#define char_delay() _delay_ms(10) // char delay
#define nloop 100 // loops to accumulate

//this is for serial connection to the python script
//#define serial_port PORTB
//#define serial_direction DDRB
//#define serial_pin_out (1 << PB1)
#define transmit_port PORTA
#define transmit_direction DDRA
#define transmit_pin (1 << PA4)


void put_char(volatile unsigned char *port, unsigned char pin, char txchar) {
   //
//start bit
   //
   clear(*port,pin);
   bit_delay();
// unrolled loop to write data bits
   //
   if bit_test(txchar,0)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,1)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,2)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,3)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,4)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,5)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,6)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   if bit_test(txchar,7)
      set(*port,pin);
   else
      clear(*port,pin);
   bit_delay();
   //
   // stop bit
   //
   set(*port,pin);
   bit_delay();
   //
   // char delay
   //
   bit_delay();
   }
   //


int main(void) {

static unsigned char count;
   static uint16_t up,down,diff;
//
  unsigned char count, pwm;
   //
   // set clock divider to /1
   //
   CLKPR = (1 << CLKPCE);
   CLKPR = (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
   //
   // initialize output pins
   //don't need the first piece because this is where in the original code it was talking to python script
   // set(serial_port, serial_pin_out);
  // output(serial_direction, serial_pin_out);
   clear(transmit_port, transmit_pin);
   output(transmit_direction, transmit_pin);
   //
   //
   // initialize RGB LED pins
   set(led_port, red);
   output(led_direction, red);
   set(led_port, green);
   output(led_direction, green);
   set(led_port, blue);
   output(led_direction, blue);


 
   //
 // init A/D
   //
  
   ADMUX = (0 << REFS1) | (0 << REFS0) // Vcc ref
      | (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0); // PA7
   ADCSRA = (1 << ADEN) // enable
      | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); // prescaler /128
   //
   // main loop
   //
   while (1) {
      //
      // accumulate
      //
      up = 0;
      down = 0;
      //comment out for loop and then the cummulative nature of up and down
      //for (count = 0; count < nloop; ++count) { 
         //
         // settle, charge
         //
         settle_delay();
         set(transmit_port, transmit_pin);
         //
         // initiate conversion
         //
         ADCSRA |= (1 << ADSC);
         //
         // wait for completion
         //
         while (ADCSRA & (1 << ADSC))
            ;
         //
         // save result
         //
         //up += ADC;
         up = ADC;
         // settle, discharge
         //
         settle_delay();
         clear(transmit_port, transmit_pin);
         //
         // initiate conversion
         //
         ADCSRA |= (1 << ADSC);
         //
         // wait for completion
         //
         while (ADCSRA & (1 << ADSC))
            ;
         //
         // save result
         //
         //down += ADC;
         down = ADC;
         }

         diff = up - down
   //
   //
    //    put_char(&serial_port, serial_pin_out, 1);
     // char_delay();
      //put_char(&serial_port, serial_pin_out, 2);
      //char_delay();
      //put_char(&serial_port, serial_pin_out, 3);
      //char_delay();
      //put_char(&serial_port, serial_pin_out, 4);
      //
      // send result
      //
      //put_char(&serial_port, serial_pin_out, (up & 255));
      //char_delay();
      //put_char(&serial_port, serial_pin_out, ((up >> 8) & 255));
      //char_delay();
      //put_char(&serial_port, serial_pin_out, (down & 255));
      //char_delay();
      //put_char(&serial_port, serial_pin_out, ((down >> 8) & 255));
      //char_delay();
         //}

    //Diff is less than a certain value
    //Rob says that diff will be around 100-150, so we shall see what happens with this tryout experience.
   if (diff < 100) {
       for (count = 0; count < 100; ++count) {
               clear(led_port,red);
               set(led_port,green);
               set(led_port;blue);
            for (pwm = 0; pwm < count; ++pwm);
            PWM_delay()
      }}

 	else if (diff < 400) {
         for (count = 0; count < 100; ++count) {
               set(led_port,red);
               clear(led_port,green);
               set(led_port;blue);   
            for (pwm = 0; pwm < count; ++pwm);
            PWM_delay()
      }}       

//
	else (diff > 400) {
         for (count = 0; count < 100; ++count) {
               set(led_port,red);
               set(led_port,green);
               clear(led_port;blue);   
            for (pwm = 0; pwm < count; ++pwm);
            PWM_delay()
      }}       


}
//My own code. have nothing about the rgb here.


