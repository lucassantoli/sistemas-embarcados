#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>

//PORTD
const byte trigger_pin   = 0B00100000;
const byte echo_pin      = 0B00010000;

//PORTB
const byte stop_motor    = 0B00111100;
const byte fwd_motor     = 0B00101000;
const byte rev_motor     = 0B00010100;
const byte cw_motor      = 0B00100100;
const byte ccw_motor     = 0B00011000;
const byte infrared_pin  = 0B10000000;


////////////////////////////////////////////////

volatile unsigned long timer1_millis;  // faixa de valores vai de 0 a 4.294.967.295 (2^32 - 1)

ISR(TIMER1_COMPA_vect){
  timer1_millis++;
}

////////////////////////////////////////////////

void inicia_millis(unsigned long f_cpu){
  unsigned long ctc_match_overflow;
  ctc_match_overflow = ((f_cpu / 1000) / 8); // overflow do timer em 1ms
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  OCR1AH = (ctc_match_overflow >> 8);
  OCR1AL = ctc_match_overflow;
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

////////////////////////////////////////////////

unsigned long nossamillis () {
  unsigned long millis_return;
  ATOMIC_BLOCK(ATOMIC_FORCEON) {
    millis_return = timer1_millis;
  }

  return millis_return;
}

void trigger_sensor(byte port) {
  PORTB &= ~(port); // port LOW
  delayMicroseconds(2);
  PORTB |=  (port); // port HIGH
  delayMicroseconds(10);
  PORTB &= ~(port); // port LOW
}

////////////////////////////////////////////////

float detect_echo(byte port) {
  unsigned long timer_ultrasonic = 0;
  float distance = 0;

  while (!((PINB & 16) >> 4)); // do nothing
  while ((PINB & 16) >> 4) {  // object detected
    timer_ultrasonic++;
  }
  if (timer_ultrasonic > 0) {
    distance = timer_ultrasonic * 0.00945;
  }

  return distance;
}

////////////////////////////////////////////////

void girar() {
  Serial.println("Girandoooo");
  PORTD |=  (cw_motor );
  PORTD &= ~(ccw_motor);
  //PORTD = 0B00100100;
}

void andar() {
  Serial.println("Indo frente");
  PORTD |=  (fwd_motor);
  PORTD &= ~(rev_motor);
  //PORTD = 0B00101000;
}

////////////////////////////////////////////////

int main(void) {

  inicia_millis(16000000UL);
  Serial.begin(9600);

  unsigned long timer_ultrasonic = 0;
  unsigned char sensor_pino_7;
  float distance;
  int contador;
  bool podeGirar = false;
  bool taNaBorda = false;
  bool temRobo = false;
  unsigned long comeco = 0;


  DDRB |=  (trigger_pin); // pin 13 output
  DDRB &= ~(echo_pin);    // pin 12 input

  DDRD |=  (motor_pins);  // pin 2 3 4 5 output
  DDRD &= ~(data_ir_pin); // pin 7 input


  while (1) {
    unsigned long currentMillis = nossamillis();

    //ir data
    sensor_pino_7 = (PIND & infrared_pin) >> 7;
    if (sensor_pino_7 == 0) {
      taNaBorda = true;
      comeco = nossamillis();
    }

    if (taNaBorda) {
      if (nossamillis() <= comeco + 2000) {
        girar();
      }
      else {
        Serial.println("Nao girandooo");
        taNaBorda = false;
      }
    }

    //ultrasonic sensor
    trigger_sensor(trigger_pin);
    timer_ultrasonic = 0; // restart timer
    distance = detect_echo(echo_pin);

    if (!taNaBorda) {
      if (distance < 20) {
        temRobo = true;
        andar();
      } else {
        temRobo = false;
        girar();
      }
    }
  }

  return 0;
}
