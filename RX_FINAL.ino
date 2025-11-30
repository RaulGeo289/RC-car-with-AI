

// TX_fixed.c
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Pins: CE = PB0 (D9), CSN = PB2 (D10)
#define CE_HIGH()  (PORTB |=  (1<<PB0))
#define CE_LOW()   (PORTB &= ~(1<<PB0))
#define CSN_HIGH() (PORTB |=  (1<<PB2))
#define CSN_LOW()  (PORTB &= ~(1<<PB2))

void uart_init() {
    unsigned int ubrr = F_CPU/16/9600 - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}
void uart_send_chars(const char* s){
    while (*s) uart_send(*s++);
}
void uart_print_bin(uint8_t val) {
    const char bits[] = "01";
    for (int i = 7; i >= 0; --i) uart_send(bits[(val >> i) & 1]);
}

// SPI
void spi_init() {
    DDRB |= (1<<PB3) | (1<<PB5) | (1<<PB2); // MOSI, SCK, CSN outputs
    DDRB &= ~(1<<PB4);                      // MISO input
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); // SPI enable, master, f/16
}
uint8_t spi_transfer(uint8_t data){
    SPDR = data;
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

// nRF helpers
void nrf_write_reg(uint8_t reg, uint8_t val){
    CSN_LOW();
    spi_transfer(0x20 | (reg & 0x1F));
    spi_transfer(val);
    CSN_HIGH();
}
void nrf_write_reg_multi(uint8_t reg, const uint8_t* buf, uint8_t len){
    CSN_LOW();
    spi_transfer(0x20 | (reg & 0x1F));
    for(uint8_t i=0;i<len;i++) spi_transfer(buf[i]);
    CSN_HIGH();
}
uint8_t nrf_read_reg(uint8_t reg){
    CSN_LOW();
    spi_transfer((0x00 | (reg & 0x1F)));
    uint8_t v = spi_transfer(0xFF);
    CSN_HIGH();
    return v;
}

void PWMsetup(uint8_t dutyCycle){  //dutyCycle val= intensity of the motor,for both DC motors and servo motor
    DDRD |= (1 << PD3) ; // D3=DC motors,D6=servo motor 
TCCR2A = (1<<COM2A1) | (1 << COM2B1) | (1 << WGM20); 
TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler = 1024
    OCR2B = dutyCycle; 
}




// call once in setup
void servo_init() {
    DDRB |= (1 << PB1);            // D9 / OC1A output

    // Fast PWM, TOP = ICR1, non-inverting on OC1A (Mode 14)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11); // prescaler = 8

    ICR1 = 39999;   // TOP -> 50 Hz
}

// write pulse in microseconds (1000..2000 typical)
void servo_write_us(uint16_t pulse_us) {
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;
    OCR1A = (uint16_t)(pulse_us * 2); // 2 ticks per µs
}




int main(){
    servo_init();
    DDRB |= (1<<PB0);   // CE output
spi_init();
uint8_t addresBuf[5]={0xE7,0xE7,0xE7,0xE7,0xE7};
uart_init();
nrf_write_reg(0x02,0x01);//enable data pipe 0
nrf_write_reg(0x01,0x00);//disable auto-ACK on pipe 0
nrf_write_reg(0x03,0x03);//adress width of 5 bytes
nrf_write_reg(0x05,0x00);//set frq to 2.4GHz
nrf_write_reg_multi(0x0A,addresBuf,5);//write the addres bytes 
nrf_write_reg(0x11, 5); // expect 5 bytes per packet

nrf_write_reg(0x00,0x00);//START at a known state,which is closed
nrf_write_reg(0x00,0x03);//set PWR_UP and PRIM_RX bits
_delay_ms(5);
    uint8_t statusRegVal=nrf_read_reg(0x00);
    uart_send_chars("CONFIG reg val,after setting it , is:");
    uart_print_bin(statusRegVal);

CE_HIGH();//START LISTENING

DDRD|=(1<<PD4) | (1<<PD5);//set on output for the motor

uint8_t motorIntensity;//the value fed to the motor

uint8_t gotStarting=0;
while(gotStarting==0){
    do{
    statusRegVal=nrf_read_reg(0x07);
    }
    while(!(statusRegVal & (1<<6)));//wait until the signal is received
    //just wait for the starting byte
    CSN_LOW();//get value from RX FIFO
    spi_transfer(0x61);//
    uint8_t fifth=spi_transfer(0xFF);//higher half of the signal for x axis
    uint8_t fourth=spi_transfer(0xFF);//lower half of the signal for x axis
    uint8_t third=spi_transfer(0xFF);//higher half of the servo (y axis)
    uint8_t second=spi_transfer(0xFF);//lower half of the servo (y axis)
    uint8_t first=spi_transfer(0xFF);//switch val
    CSN_HIGH();//
    uint16_t ff=fifth;
    ff=ff<<8;
    ff+=fourth;
    uint16_t ts=third;
    ts=ts<<8;
    ts+=second;
    if(ff==0xABBA && ts==0x0000 && first==0x01){
        gotStarting=1;
        CE_LOW();
        nrf_write_reg(0x00,0x02);//set on tx mode and start listening
        _delay_ms(5);
    }
    uart_send_chars("Not receiveing the right data\n");
}   
for(int i=0;i<5;i++){
    //after you gor it,the RX send another starting byte to TX to signal that it is ready for getting the important information
    //CONFIGURE AS tx
    uart_send_chars("Sending...");
    CSN_LOW();
    nrf_write_reg(0x00,0x02);
    _delay_ms(5);

        //load starting byte
    CSN_LOW();
    spi_transfer(0xA0);
    spi_transfer(0xAB);
    spi_transfer(0xBA);
    spi_transfer(0x00);
    spi_transfer(0x00);
    spi_transfer(0x01);
    CSN_HIGH();

    uart_send_chars("\n");
            // //send payload
    CE_HIGH();
    _delay_us(20);
    CE_LOW();
    uart_send_chars("\n");
    uart_send_chars("Transfer Complete");
    uart_send_chars("\n");
        while(!(statusRegVal & (1<<5))){
            statusRegVal=nrf_read_reg(0x07);
            if(statusRegVal & (1<<4)){  //if max_rt bit = 1,flush TX and retry
            //flush TX FIFO
                uart_send_chars("MAX_RT flag!!");
                CSN_LOW();
                spi_transfer(0xE1);
                spi_transfer(0xFF);
                CSN_HIGH();
                _delay_us(10);//wait for flush
            //retry
                CSN_LOW();
                spi_transfer(0xA0);
                spi_transfer(0xAB);
                spi_transfer(0xBA);
                spi_transfer(0x00);
                spi_transfer(0x00);
                spi_transfer(0x01);
                CSN_HIGH();
            }
            uart_send_chars("Data not sent!TX_DS flag not set\n");
            _delay_ms(500);
        }
        uart_send_chars("Data sent for starting byte!!");

}
    nrf_write_reg(0x00,0x03);
    _delay_ms(5);

while(1){
        //flush RX FIFO
    CSN_LOW();
    spi_transfer(0xE2);
    spi_transfer(0xFF);
    CSN_HIGH();
    do{
    statusRegVal=nrf_read_reg(0x07);
    }
    while(!(statusRegVal & (1<<6)));//wait until the signal is received
    CSN_LOW();//get value from RX FIFO
    spi_transfer(0x61);//
    uint8_t Hservo=spi_transfer(0xFF);//higher half of the signal for x axis
    uint8_t Lservo=spi_transfer(0xFF);//lower half of the signal for x axis
    uint8_t Hval=spi_transfer(0xFF);//higher half of the servo (y axis)
    uint8_t Lval=spi_transfer(0xFF);//lower half of the servo (y axis)
    uint8_t sw=spi_transfer(0xFF);//switch val
    CSN_HIGH();//

    uint16_t valServo=Hservo;//x axis
    valServo=valServo<<8;
    valServo+=Lservo;

    uint16_t val=Hval;//y axis
    val=val<<8;
    val+=Lval;

    nrf_write_reg(0x07, (1<<6));//clear the RX_DS bit

    motorIntensity=val>>2;//transform val in a 8 bit value
    valServo+=1000;//map the servo value from 0->1023 to 1000->2023 for the serco_write_us func
    if (motorIntensity<=120){//reverse polarity for the motor       
        PORTD&=~(1<<PD4);//
        PORTD|=(1<<PD5);//
        PWMsetup(240-2*motorIntensity);
    }
    else if(motorIntensity>120 && motorIntensity<135){//the motor is stopped         
        PORTD&=~(1<<PD4);//
        PORTD|=(1<<PD5);// 
        PWMsetup(0);
    }
    else if(motorIntensity>=135){//motor is moving forward       
        PORTD|=(1<<PD4);//
        PORTD&=~(1<<PD5);//
        PWMsetup(motorIntensity);
    }

    if(valServo>=1006 && valServo<=2000)//steering mechanism
        servo_write_us(valServo);
    else if (valServo>2000)
        servo_write_us(2500);
    else if(valServo<=1005)
    servo_write_us(500);

    //if sw is 1 ....
    //else ....


    uart_send_chars("Intensitatea motorului este");
    uart_print_bin(motorIntensity);
    uart_send_chars("\n");

}
}

