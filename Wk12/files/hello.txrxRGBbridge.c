// Manar-ul Islam Swaby
// 12/ 2016
// hello.txrxRGBbus.44.c
//
//this board is the bridge for the network
// serial bus communication step response transmit-receive rgb led hello-world
//    9600 baud FTDI interface
//
// RGB LED software PWM hello-world
//
//
#include <avr/io.h>
#include <util/delay.h>
//these two bottom libraries are specific to serial busing
#include <avr/pgmspace.h>
#include <string.h>

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

//bus.44
//#define output(directions,pin) (directions |= pin) // set port direction for output
//#define input(directions,pin) (directions &= (~pin)) // set port direction for input
//#define set(port,pin) (port |= pin) // set port pin
//#define clear(port,pin) (port &= (~pin)) // clear port pin
//#define pin_test(pins,pin) (pins & pin) // test for port pin
//#define bit_test(byte,bit) (byte & (1 << bit)) // test for bit set
//#define bit_delay_time 100 // bit delay for 9600 with overhead
//#define bit_delay() _delay_us(bit_delay_time) // RS232 bit delay
//#define half_bit_delay() _delay_us(bit_delay_time/2) // RS232 half bit delay
#define led_delay() _delay_ms(100) // LED flash delay

//commented out because I used an rgb led and port specifics are above
//#define led_port PORTB
//#define led_direction DDRB
//#define led_pin (1 << PB0)

#define serial_port PORTB
#define serial_direction DDRB
#define serial_pins PINB
#define serial_pin_in (1 << PB1)
#define serial_pin_out (1 << PB0)

#define node_id '0'


//copied from bus.45
void get_char(volatile unsigned char *pins, unsigned char pin, char *rxbyte) {
   //
   // read character into rxbyte on pins pin
   //    assumes line driver (inverts bits)
   //
   *rxbyte = 0;
   while (pin_test(*pins,pin))
      //
      // wait for start bit
      //
      ;
   //
   // delay to middle of first data bit
   //
   half_bit_delay();
   bit_delay();
   //
   // unrolled loop to read data bits
   //
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 0);
   else
      *rxbyte |= (0 << 0);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 1);
   else
      *rxbyte |= (0 << 1);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 2);
   else
      *rxbyte |= (0 << 2);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 3);
   else
      *rxbyte |= (0 << 3);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 4);
   else
      *rxbyte |= (0 << 4);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 5);
   else
      *rxbyte |= (0 << 5);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 6);
   else
      *rxbyte |= (0 << 6);
   bit_delay();
   if pin_test(*pins,pin)
      *rxbyte |= (1 << 7);
   else
      *rxbyte |= (0 << 7);
   //
   // wait for stop bit
   //
   bit_delay();
   half_bit_delay();
   }


//copied from bus but also exists within txrx
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
//copied from bus
void put_string(volatile unsigned char *port, unsigned char pin, PGM_P str) {
   //
   // send character in txchar on port pin
   //    assumes line driver (inverts bits)
   //
   static char chr;
   static int index;
   index = 0;
   do {
      chr = pgm_read_byte(&(str[index]));
      put_char(&serial_port, serial_pin_out, chr);
      ++index;
      } while (chr != 0);
   }
//copied from bus
void flash() {
   //
   // LED flash delay
   //
   clear(led_port, blue);
   clear(led_port, green);
   clear(led_port, red);
   led_delay();
   set(led_port, blue);
   set(led_port, green);
   set(led_port, red);
   }

int main(void) {
   //
   //main doings of the board
   //
   //rgb
   unsigned char count, pwm;
   //txrx
   static unsigned char count;
   static uint16_t up,down,diff;
    //bus
   static char chr;
    //
   // set clock divider to /1
   //
   CLKPR = (1 << CLKPCE);
   CLKPR = (0 << CLKPS3) | (0 << CLKPS2) | (0 << CLKPS1) | (0 << CLKPS0);
   //
   //
   // RGB initializeLED pins
   set(led_port, red);
   output(led_direction, red);
   set(led_port, green);
   output(led_direction, green);
   set(led_port, blue);
   output(led_direction, blue);
   //
   // txrx initialize output pins
   //don't need the first piece because this is where in the original code it was talking to python script
   // set(serial_port, serial_pin_out);
   // output(serial_direction, serial_pin_out);
   clear(transmit_port, transmit_pin);
   output(transmit_direction, transmit_pin);
   //
   // bus initialize output pins
   set(serial_port, serial_pin_out);
   input(serial_direction, serial_pin_out);
 
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
      //txrx
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

    //rgb
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


//specific to bus and communicating the values
      get_char(&serial_pins, serial_pin_in, &chr);
      flash();
      if (chr == node_id) {
         output(serial_direction, serial_pin_out);
         static const char message[] PROGMEM = "node ";
         put_string(&serial_port, serial_pin_out, (PGM_P) message);
         put_char(&serial_port, serial_pin_out, chr);
         put_char(&serial_port, serial_pin_out, 10); // new line
         //diff is not necessarily integrated in this line of code. Will obviously work when connected to my c code that holds diff as a variable.
         //put_char(&serial_port, serial_pin_out, diff); 
         //put_char(&serial_port, serial_pin_out, 10); // new line
         led_delay();
         flash();
         input(serial_direction, serial_pin_out);
         }
}



