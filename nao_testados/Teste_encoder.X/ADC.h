/*---------------------------------------------------------------
 * BIBLIOTECA PARA UTILIZACAO DO CONVERSOR AD DO AVR
 * Modificada por: PROF. RODRIGO RECH
 * 08/2019
 * -----------------------------------------------------------------*/

void inicializa_ADC(void){
    
    //Habilita a referência de tensão interna do ADC e ativa o canal 0 (VCC)
    ADMUX = (1<<REFS0);                   
    
    //Habilita o ADC e configura o prescaler para 128
    ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

unsigned int le_ADC(unsigned char canal){
    
    canal = canal & 0b00001111;
    ADMUX = (ADMUX & 0xF0) | canal; 

    //Inicia a conversão
    ADCSRA |= (1 << ADSC);

    //Aguarda a finalização da conversão
    while ( (ADCSRA & (1<<ADSC)));
    
    //Retorna o valor convertido
    return ( ADC ); 
}
