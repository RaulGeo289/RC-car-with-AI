// TX_fixed.c
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Pins: CE = PB1 (D9), CSN = PB2 (D10)
#define CE_HIGH()  (PORTB |=  (1<<PB1))
#define CE_LOW()   (PORTB &= ~(1<<PB1))
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


//the next functions are for working with the joystick,the motordriver and the motors
void ADCsetup(){
    ADCSRA|=(1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
    PRR|=(0<<PRADC);//disable power reduction ADC
    ADMUX|=(1<<REFS0);//voltage refference: 5V
}

uint16_t adc_read(uint8_t channel){
    ADMUX=(ADMUX & 0xF0) | (channel & 0x0F);//clear ADMUX reg, only lower half ,and add channel bits

    ADCSRA|=(1<<ADSC);//start conversion with 1 on ADSC bit 

    while(ADCSRA & (1<<ADSC));

    return ADC;//return the value from the result register
}

uint8_t digiRead(uint8_t port,uint8_t pin){
    switch(port){
        case 'B' :return (PINB & (1<<pin));
        case 'C' :return (PINC & (1<<pin));
        case 'D' :return (PIND & (1<<pin)); 
    }
}

uint8_t switchRead(uint8_t port,uint8_t pin){
    uint8_t sw=digiRead(port,pin);
    if (sw){
        uart_send_chars("Switch pressed!");
    }
    else{
        uart_send_chars("Switch not pressed!");
    }
}

void printReg(uint8_t reg){
        uint8_t r1=nrf_read_reg(reg);
        uart_print_bin(reg);
        uart_send_chars(" reg:");
        uart_print_bin(r1);
        uart_send_chars("\n");
}

int main(){
uint8_t count=0;
DDRB |= (1<<PB1);   // CE output
spi_init();
uart_init();
uint8_t addresBuf[5]={0xE7,0xE7,0xE7,0xE7,0xE7};
nrf_write_reg(0x02,0x01);//enable data pipe 0
nrf_write_reg(0x01,0x00);//disable auto-ACK on pipe 0
//tx mode setup
nrf_write_reg(0x00,0x00);//START at a known state,which is closed state
nrf_write_reg(0x00,0x02);//set PWR_UP and PRIM_RX bits
_delay_ms(5);
 uint8_t statusRegVal=nrf_read_reg(0x07);

//flush TX FIFO
CSN_LOW();
spi_transfer(0xE1);
spi_transfer(0xFF);
CSN_HIGH();

nrf_write_reg(0x04,0x0F);//set on 15 retries
nrf_write_reg(0x03,0x03);//adress width of 5 bytes
nrf_write_reg(0x05,0x00);//set frq to 2.4GHz
nrf_write_reg_multi(0x10,addresBuf,5);//write the addres bytes 
nrf_write_reg_multi(0x0A, addresBuf, 5); // RX_ADDR_P0
nrf_write_reg(0x11,0x05);//set TX payload to 5 bytes
uint8_t configRegVal;
    DDRC|=(0<<PC1) | (0<<PC0); //set pins A0 and A1 as inputs
    DDRD &= ~(1<<PD2);   // pin 2 as input
PORTD |= (1<<PD2);  // enable pull-up
    uint16_t x,y;
    uint8_t sw;
    ADCsetup();
    uint8_t r1,r2,r3;
uart_send_chars("\n");
printReg(0x07);
uint8_t gotStarting=0;
//////////// SENDING STARTING BYTE
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

        uart_send_chars("Data sent! TX_DS flag reset\n");
while(gotStarting==0){
        uart_send_chars("Waiting for package from receiver to send the data!");
        nrf_write_reg(0x00,0x03);//set on rx mode and start listening
        _delay_ms(5);
        CE_HIGH();//START LISTENING
        _delay_ms(15);
        //flush RX FIFO
        CSN_LOW();
        spi_transfer(0xE2);
        spi_transfer(0xFF);
        CSN_HIGH();
        do{
            configRegVal=nrf_read_reg(0x00);
            statusRegVal=nrf_read_reg(0x07);
            uart_send_chars("CONFIG REG VAL IS:");
            uart_print_bin(configRegVal);
        }
        while(!(statusRegVal & (1<<6)));//wait until the signal is received
        uart_send_chars("data received!");
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
        if(ff==0xabba && ts==0x0000 && first==0x01){
            uart_send_chars("Got starting byte!!");
            gotStarting=1;
            CE_LOW();
            nrf_write_reg(0x00,0x02);//set on tx mode and start listening
            _delay_ms(5);
        }
}
nrf_write_reg(0x00,0x02);//set on tx mode and start listening
            _delay_ms(5);

/////////////
    while(1){
            //joystick part
        x=adc_read(0);//read x axis
        y=adc_read(1);//read y axis
        sw=digiRead(PORTD,2);//read button state
        
        //debug with UART
        uart_send_chars("X is:");
        uart_print_bin(x>>8);
        uart_print_bin(x);
        uart_send_chars("\n");

        uart_send_chars("Y is:");
        uart_print_bin(y>>8);
        uart_print_bin(y);
        uart_send_chars("\n");

        uart_send_chars("Button state is:");
        uart_print_bin(sw);
        uart_send_chars("\n");
        

//flush TX
    CSN_LOW();
    spi_transfer(0xE1);
    spi_transfer(0xFF);
    CSN_HIGH();

            // //load payload
            CSN_LOW();
            spi_transfer(0xA0);
            spi_transfer(x>>8);
            spi_transfer(x);
            spi_transfer(y>>8);
            spi_transfer(y);
            spi_transfer(sw);
            CSN_HIGH();
uart_send_chars("\n");
            // //send payload
            CE_HIGH();
            _delay_us(20);
            CE_LOW();
uart_send_chars("\n");
            uart_send_chars("Transfer Complete");
            uart_send_chars("\n");
        //NRF transmitter part
    statusRegVal=nrf_read_reg(0x07);
    uart_send_chars("STATUS reg val is:");
    uart_print_bin(statusRegVal);
    uart_send_chars("\n");///////////////////////////////////////////////////////////////////

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
            spi_transfer(x>>8);
            spi_transfer(x);
            spi_transfer(y>>8);
            spi_transfer(y);
            spi_transfer(sw);
            CSN_HIGH();
        }
        uart_send_chars("Data not sent!TX_DS flag not set\n");
        _delay_ms(500);
    }

    uart_send_chars("Data sent! TX_DS flag reset\n");
    }
 }

