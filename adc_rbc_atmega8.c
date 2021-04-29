//============================================== Telecommunications University===============================
// Author : DoNgocTuan	/ Telecommunications tech department                                                        
// Email:   ngoctuansqtt@gmail.com	
// Phone:   0986588017												 		
// Date :   21/12/2016                                                        
// Version: 1.1                                                                                                                                 
// Description: Robocon2017 module ADC follow line
// ============================================== make myselft===================================     

// Vesion 1.1


#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

 
#define ADC_VREF_TYPE 0x20
uint8_t eeprom_read_byte (const uint8_t *addr) ;


#define	AREF_MODE	0						//dien ap tham chieu ngoai, dien ap tren chan Vref
#define	AVCC_MODE	(1<<REFS0)				//dung dien ap chan AVcc lam dien ap tham chieu, chan Vref gan voi 1 tu dien 
#define	INT_MODE	(1<<REFS1)|(1<<REFS0)	//dung dien ap tham chieu noi 2.56V,  chan Vref gan voi 1 tu dien
#define ADC_VREF_TYPE AVCC_MODE        		//dinh nghia dien ap tham chieu

#define 	sbi(sfr,bit)	sfr|=_BV(bit)
#define 	cbi(sfr,bit)	sfr&=~(_BV(bit))


uint16_t read_adc(unsigned char adc_channel)
{//chuong trinh con doc ADC theo tung channel	
    ADMUX=adc_channel|ADC_VREF_TYPE;
    ADCSRA|=(1<<ADSC); 					//bat dau chuyen doi             
	loop_until_bit_is_set(ADCSRA,ADIF); //cho den khi nao bit ADIF==1  
    return ADCW;
}
 
uint16_t ADC_val[8];                                                  
unsigned char  w[8] = {1,2,4,8,16,32,64,128};
unsigned int  sensor;
uint16_t temp_adc[8]={0,0,0,0,0,0,0,0} ;  // mang rom luu gia tri lay ma so sanh
uint16_t mau_adc[8]={0,0,0,0,0,0,0,0} ;  // mang rom luu gia tri lay ma so sanh
uint16_t value_rom[8]={0,0,0,0,0,0,0,0} ;
uint16_t value_trang[8]={0,0,0,0,0,0,0,0} ;

void value_trang_luu()
{
	char j;
	for(j=0;j<8;j++)
	{
	value_trang[j]= read_adc(j);
	}
}

void ghi_adc_in_rom (void) // chuong trinh thuc hien ghi ADC vao ROM
{  
      uint8_t i,x; 
	   for(i=0;i<8;i++)
	   {
		x=46+2*i;
	   value_rom[i]= (ADC_val[i]+ value_trang[i])/2;
	   eeprom_write_word((uint16_t *)x,value_rom[i] ); 
       _delay_ms(10);    // thuc hien tre thoi gian ghi
	 
	   }
  
}
 
void doc_rom(void)    // thuc hien viec doc
{
    uint8_t j,x; 

    for(j=0;j<8;j++)
    {
         x=46+2*j;
		 ADC_val[j] = read_adc(j);
		 temp_adc[j] = eeprom_read_word((uint16_t *)x) ;
		 mau_adc[j]= temp_adc[j];
	}
	 return mau_adc;
}


// ha
void doc_adc(void)    // thuc hien viec doc
{
    uint8_t j; 
	sensor = 0x00;	
    for(j=0;j<8;j++)
    {
        
		 ADC_val[j] = read_adc(j);		
		if((mau_adc[j]) < ADC_val[j])  // kiem tra so sanh voi gia tri ADC mau
        {           
			 char t;
		   for(t=0;t<5;t++)
		   {
		   _delay_ms(1);
		   if((mau_adc[j]) >= ADC_val[j]) goto thoat;
		   }
			
			sensor |= w[7-j];  // thuc hien cong don gia tri 
			thoat: ;
        }               
        else sensor |= 0;
    }

     PORTD=sensor;
	
}

// ham bao hieu luu ROM bang led
void led_save_rom()
{
PORTD=0xFF;
char i;
for(i=0;i<8;i++)
{
cbi(PORTD,i);
_delay_ms(50);
}
PORTD=0xFF;
for(i=0;i<3;i++)
{
PORTD=0;
_delay_ms(50);
PORTD=0xFF;
_delay_ms(50);
}
}



void led_save_value_trang()
{
PORTD=0xFF;
char i;
for(i=8;i>1;i--)
{
cbi(PORTD,i);
_delay_ms(50);
}
PORTD=0xFF;
}

// ham luu gia tri vao ROM

void save_rom()
{  	
			_delay_ms(2);
			if(bit_is_set(PINB,0)) goto out_noise;  
			_delay_ms(2);
			if(bit_is_set(PINB,0)) goto out_noise;
			_delay_ms(2);
			if(bit_is_set(PINB,0)) goto out_noise;
			_delay_ms(2);
			if(bit_is_set(PINB,0)) goto out_noise;
			_delay_ms(2);
			if(bit_is_set(PINB,0)) goto out_noise;                        
			while(bit_is_clear(PINB,0));
			ghi_adc_in_rom(); 
		    led_save_rom(); 
			out_noise: ;      
}

// ham doc gia tri ROM de so sanh


int main(void)
{

//start ADC=============//
ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS0); 	//enable ADC, khong dung interrupt
ADMUX=ADC_VREF_TYPE; 						//chon kieu dien ap tham chieu    
//======================//


 
PORTC = 0X00; // PORT nhan ADC
DDRC = 0x00;  // PORT nhan ADC

PORTD = 0xFF; // PORT xuat tin hieu den MCU
DDRD = 0xFF;  // PORT xuat tin hieu den MCU

PORTB = 0xFF;  // PORT nhan nut nhan
DDRB =0;

doc_rom();

while (1)
{
			if(bit_is_clear(PINB,0))
        		{  
       				 save_rom();
					 _delay_ms(100);
					 doc_rom();
				}

		   if(bit_is_clear(PINB,1)) // hien thi gia tri adc luu ROM
		       
                { 
				while (bit_is_clear(PINB,1));
			     value_trang_luu();	 
				led_save_value_trang();
        		}

        	doc_adc();
    	      
} 
      
} // main
          


  
	





