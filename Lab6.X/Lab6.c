#include <xc.h>
#include <stdio.h>
#define _XTAL_FREQ 1000000
#include "LibLCDXC8.h"

#pragma config FOSC=INTOSCIO_EC
#pragma config WDT=OFF
#pragma config LVP=OFF

#define TRIGGER RC0
#define ECHO    RC1

unsigned char Estrella[8] = {
    0b00100,   
    0b01110,  //  *** 
    0b11111,  // *****
    0b01110,  //  *** 
    0b11111,  // *****
    0b01110,  //  *** 
    0b00100,  //   *  
    0b00000   // vacío
};

unsigned char RayaAlPiso[]={
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b11111
};

unsigned int Supercontador;
unsigned int contador;
unsigned int contadorRGB;
unsigned char posicion;
unsigned char editor;
unsigned int Objetivo;
unsigned char salir;
unsigned char Tecla;
unsigned char Pulsado;
unsigned char Inactividad;
unsigned int valorADC;
unsigned int valorADC2;
unsigned char datoSerial;
unsigned char d;
unsigned char etimeout=0,ctimeout=0;
unsigned char porcentajeADC;
unsigned int anchoPulso=0;
unsigned int valorGuardado=0;
unsigned int objetivoGuardado=0;

unsigned char modoReversa = 0;          // 0=avance, 1=reversa

// --- PWM manual tipo ?código bueno? ---
unsigned char control_manual_pwm = 0;   // 0=ADC, 1=manual por %
unsigned char pwm_duty_perc = 0;        // 0..100

void __interrupt()ISR(void);
void ConfigVariables(void);
void Bienvenida(void);
void PreguntaAlUsuario(void);
void ConfigPregunta(void);
void Borrar(void);
unsigned int Conversion(unsigned char);
void putch(char);
unsigned char MedirDistancia(void);
unsigned char EEPROM_Read(unsigned char);
void EEPROM_Write(unsigned char, unsigned char);
void GuardarContador(unsigned int, unsigned int);
unsigned int LeerObjetivo(void);
unsigned int LeerContador(void);

void Motor_Avance(void);
void Motor_Reversa(void);
void Motor_Paro(void);

// ====== NUEVO: setters PWM 10-bit (CCPR1L + DC1B1/DC1B0) ======
static inline void PWM_Set10bit(unsigned int duty10)
{
    if(duty10 > 1023) duty10 = 1023;
    CCPR1L = (unsigned char)(duty10 >> 2);
    CCP1CONbits.DC1B1 = (duty10 >> 1) & 1;
    CCP1CONbits.DC1B0 = (duty10 >> 0) & 1;
}

static inline void PWM_SetPercent(unsigned char perc)
{
    if(perc > 100) perc = 100;
    // 0..100% -> 0..1023
    unsigned int val_reg = (unsigned int)((unsigned long)perc * 1023UL / 100UL);
    PWM_Set10bit(val_reg);
}

void main (void){

    ConfigVariables();
    editor=0;
    datoSerial=0;

    OSCCON = 0b01000000;

    ADCON1=0b001110;

    // PWM (CCP1 + Timer2)
    TRISC2=0;
    PR2=249;
    CCPR1L = 125;
    T2CON=0b00000000;
    CCP1CON =0b00001100;
    TMR2=0;
    T2CONbits.TMR2ON=1;

    // Reversa (RA6)
    TRISA6 = 0;
    LATA6  = 0;
    modoReversa  = 0;

    ADCON0=0b00000001;
    ADCON2=0b10001000;

    TRISC0=0;
    TRISC1=1;
    LATC0=0;
    T1CON=0b10010000;

    TRISE=0;
    LATE=0b00000111;

    TRISD=0;
    LATD=contador;

    TRISA1=0;
    LATA1=0;

    TRISA2=0;
    LATA2=0;

    TRISA3=0;
    LATA3=0;
    TRISA4=0;

    // UART
    TRISC6=0;
    TRISC7=1;
    TXSTA=0b00100100;
    RCSTA = 0b10010000;
    BAUDCON=0b00001000;
    SPBRGH = 0;               // ====== NUEVO: por BRG16 ======
    SPBRG=25;

    TRISC1=1;

    TRISA5=0;
    LATA5=1;

    // Timer0
    T0CON=0b00000001;
    TMR0=3036;
    TMR0IF=0;
    TMR0IE=1;
    TMR0ON=1;

    // Timer1 (ultrasonido)
    T1CON=0b10000000;

    // Teclado
    TRISB=0b11110000;
    LATB=0b00000000;
    RBPU=0;
    __delay_ms(100);
    RBIF=0;
    RBIE=1;

    PEIE=1;
    GIE=1;
    RCIE=1;

    LATA3=1;
    Bienvenida();

    if(POR==0){
        Inactividad=0;
        POR=1;
        valorGuardado = LeerContador();
        objetivoGuardado = LeerObjetivo();

        BorraLCD();
        OcultarCursor();
        MensajeLCD_Var("    FALLA DE");
        DireccionaLCD(0xC0);
        MensajeLCD_Var("     ENERGIA");
        __delay_ms(2000);

        if(valorGuardado > 0 && valorGuardado <= 59 && objetivoGuardado > 0){
            Inactividad=0;
            BorraLCD();
            MensajeLCD_Var("Continuar desde");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("   cuenta: ");
            EscribeLCD_n8(valorGuardado, 2);
            __delay_ms(5000);

            BorraLCD();
            MensajeLCD_Var("1=SI   2=NO");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("Seleccione:");

            Tecla = '\0';
            while(Tecla != 1 && Tecla != 2){}

            if(Tecla == 1){
                Supercontador = valorGuardado;
                Objetivo = objetivoGuardado;

                contadorRGB = Supercontador / 10;
                contador = Supercontador - (contadorRGB * 10);

                LATD = contador;

                if(contadorRGB==0)      LATE=0b00000001;
                else if(contadorRGB==1) LATE=0b00000101;
                else if(contadorRGB==2) LATE=0b00000100;
                else if(contadorRGB==3) LATE=0b00000110;
                else if(contadorRGB==4) LATE=0b00000010;
                else if(contadorRGB==5) LATE=0b00000000;

                BorraLCD();
                MensajeLCD_Var("Cuenta");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("restaurada");
                __delay_ms(1500);
                BorraLCD();

                goto inicio_conteo;

            } else {
                ConfigVariables();
                EEPROM_Write(0b00000000, 0b00000000);
                EEPROM_Write(0b00000001, 0b00000000);
                BorraLCD();
                MensajeLCD_Var("Cuenta");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("reiniciada");
            }
        }

    }else{
        BorraLCD();
        OcultarCursor();
        MensajeLCD_Var("    RESET DE");
        DireccionaLCD(0xC0);
        MensajeLCD_Var("     USUARIO");
        __delay_ms(1000);

        ConfigVariables();
        EEPROM_Write(0b00000000, 0b00000000);
        EEPROM_Write(0b00000001, 0b00000000);
    }

    __delay_ms(1000);
    LATA3=0;

    while(1){
        PreguntaAlUsuario();

        inicio_conteo:

        OcultarCursor();
        MensajeLCD_Var("Faltantes: ");
        EscribeLCD_n8(Objetivo-Supercontador,2);
        DireccionaLCD(0xC0);
        MensajeLCD_Var("Objetivo: ");
        EscribeLCD_n8(Objetivo,2);
        salir=1;

        while (salir==1){
            __delay_ms(100);
            d=MedirDistancia();

            if(Supercontador==Objetivo){
                LATA2=1; __delay_ms(1000); LATA2=0;

                BorraLCD();
                MensajeLCD_Var("Cuenta Cumplida");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("   Presione OK");

                salir = 0;
                Tecla='\0';
                while(Tecla!= '*'){}
                ConfigVariables();
            }

            if((d>4 && d<9) && Supercontador!=Objetivo){
                Inactividad=0;
                Pulsado=0;
            }

            if(Pulsado==0){
                if((d<4 || d>9)){
                    Pulsado=1;
                    contador++;
                    Supercontador++;
                    GuardarContador(Supercontador, Objetivo);

                    if (contador==10){
                        LATA2=1; __delay_ms(300); LATA2=0;
                        contador=0;
                        contadorRGB++;
                        if (contadorRGB==6){
                            LATA2=1; __delay_ms(300); LATA2=0;
                            contadorRGB=0;
                        }
                    }

                    if(contadorRGB==0)      LATE=0b00000010;
                    else if(contadorRGB==1) LATE=0b00000011;
                    else if(contadorRGB==2) LATE=0b00000001;
                    else if(contadorRGB==3) LATE=0b00000101;
                    else if(contadorRGB==4) LATE=0b00000100;
                    else if(contadorRGB==5) LATE=0b00000000;

                    DireccionaLCD(0x8B);
                    EscribeLCD_n8(Objetivo-Supercontador,2);

                    LATD=contador;
                    __delay_ms(500);
                }
            }
        }

        LATE=0b00000001;
        LATD=contador;
    }
}

void __interrupt()ISR(void){

    if (RCIF==1){

        if(RCSTAbits.OERR){
            RCSTAbits.CREN = 0;
            RCSTAbits.CREN = 1;
        }

        Inactividad=0;
        datoSerial = RCREG;

        // ---- PWM manual por serial (LATCH) ----
        if(datoSerial=='Z' || datoSerial=='z'){ control_manual_pwm = 1; pwm_duty_perc = 0;   }
        else if(datoSerial=='X' || datoSerial=='x'){ control_manual_pwm = 1; pwm_duty_perc = 20; }
        else if(datoSerial=='C' || datoSerial=='c'){ control_manual_pwm = 1; pwm_duty_perc = 40; }
        else if(datoSerial=='V' || datoSerial=='v'){ control_manual_pwm = 1; pwm_duty_perc = 60; }
        else if(datoSerial=='B' || datoSerial=='b'){ control_manual_pwm = 1; pwm_duty_perc = 80; }
        else if(datoSerial=='N' || datoSerial=='n'){ control_manual_pwm = 1; pwm_duty_perc = 100; }
        else if(datoSerial=='M' || datoSerial=='m'){ control_manual_pwm = 0; }

        if(modoReversa==0){
            if(control_manual_pwm==1){
                PWM_SetPercent(pwm_duty_perc);
            }
        }else{
            PWM_Set10bit(0);
        }

        if(datoSerial==80 || datoSerial==112){
            LATE=0b00000011;
            BorraLCD();
            OcultarCursor();
            MensajeLCD_Var("   PARADA DE");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("   EMERGENCIA");

            PWM_Set10bit(0);
            Motor_Paro();
            while(1){}
        }
        else if(salir==1 && (datoSerial==114 || datoSerial==82)){
            contador=0;
            Supercontador = 0;
            contadorRGB = 0;
            GuardarContador(0, Objetivo);
            LATE=0b00000010;
            DireccionaLCD(0x8B);
            EscribeLCD_n8(Objetivo-Supercontador,2);
            LATD=contador;
        }
        else if(datoSerial=='F' || datoSerial=='f'){
            Motor_Avance();
        }
        else if(datoSerial=='G' || datoSerial=='g'){
            Motor_Reversa();
        }
    }

    if(TMR0IF==1){
        Inactividad++;
        TMR0=3036;
        TMR0IF=0;
        LATA1=LATA1^1;

        valorADC=Conversion(0);
        porcentajeADC= (unsigned char)((valorADC * 100UL) / 1023UL);

        printf("Valor del ADC:%d\r\n", valorADC);

        // ====== PWM (igual al que te funcionaba: 10 bits) ======
        if(modoReversa==1){
            PWM_Set10bit(0);
        }else{
            if(control_manual_pwm==1){
                PWM_SetPercent(pwm_duty_perc);
                porcentajeADC = pwm_duty_perc;
            }else{
                PWM_Set10bit(valorADC);   // 0..1023 directo al PWM
            }
        }

        printf("Valor de PWM: %d%%\r\n",porcentajeADC);

        if (etimeout == 1) ctimeout++;
        else ctimeout = 0;
        if (ctimeout >= 2) etimeout = 0;

        if(d==0) printf("Falla en el sensor\r\n");
        else     printf("La distancia medida es: %d cm\r\n",d);

        if(Inactividad == 10){
            LATA3 = 0;
        }
        if(Inactividad >= 20){
            Sleep();
            Inactividad = 0;
            RBIF = 0;
            TMR1ON = 1;
        }

        valorADC2=valorADC;
    }

    if(RBIF==1){
        if(PORTB!=0b11110000){
            Inactividad = 0;

            LATB=0b11111110;
            if(RB4==0){
                Tecla=1;
                ConfigPregunta();
            }
            else if(RB5==0) {
                Tecla=2;
                ConfigPregunta();
            }
            else if(RB6==0){
                Tecla=3;
                ConfigPregunta();
            }
            else if(RB7==0){
                Tecla='*';
            }
            else{
                LATB=0b11111101;
                if(RB4==0){
                    Tecla=4;
                    ConfigPregunta();
                }
                else if(RB5==0) {
                    Tecla=5;
                    ConfigPregunta();
                }
                else if(RB6==0) {
                    Tecla=6;
                    ConfigPregunta();
                }
                else if(RB7==0) {
                    LATE=0b00000011;
                    BorraLCD();
                    OcultarCursor();
                    MensajeLCD_Var("   PARADA DE");
                    DireccionaLCD(0xC0);
                    MensajeLCD_Var("   EMERGENCIA");
                    PWM_Set10bit(0);
                    Motor_Paro();
                    while(1){}
                }
                else{
                    LATB=0b11111011;
                    if(RB4==0) {
                        Tecla=7;
                        ConfigPregunta();
                    }
                    else if(RB5==0) {
                        Tecla=8;
                        ConfigPregunta();
                    }
                    else if(RB6==0) {
                        Tecla=9;
                        ConfigPregunta();
                    }
                    else if(RB7==0) {
                        Borrar();
                    }
                    else{
                        LATB=0b11110111;
                        if(RB4==0){
                            contador=0;
                            Supercontador = 0;
                            contadorRGB = 0;
                            GuardarContador(0, Objetivo);
                            LATE=0b00000010;
                            if(salir==1){
                                DireccionaLCD(0x8B);
                                EscribeLCD_n8(Objetivo-Supercontador,2);
                                LATD=contador;
                            }
                        }
                        else if(RB5==0) {
                            Tecla=0;
                            ConfigPregunta();
                        }
                        else if(RB6==0) {
                            Borrar();
                            Supercontador=Objetivo;
                            contadorRGB = Objetivo/10;
                            contador = Objetivo-contadorRGB*10;

                            if(contadorRGB==0)      LATE=0b00000001;
                            else if(contadorRGB==1) LATE=0b00000101;
                            else if(contadorRGB==2) LATE=0b00000100;
                            else if(contadorRGB==3) LATE=0b00000110;
                            else if(contadorRGB==4) LATE=0b00000010;
                            else if(contadorRGB==5) LATE=0b00000000;

                            LATD=contador;
                        }
                        else if(RB7==0) {
                            LATA3=LATA3^1;
                            TMR1ON=1;
                        }
                    }
                }
            }

            LATB=0b11110000;
        }

        __delay_ms(300);
        RBIF=0;
    }
}

void ConfigVariables(void){
    Pulsado=1;
    valorADC2=0;
    contador=0;
    salir=0;
    Supercontador=0;
    contadorRGB=0;
    posicion=0;
    Objetivo=0;
    Tecla='\0';

    modoReversa = 0;
    LATA6 = 0;

    control_manual_pwm = 0;
    pwm_duty_perc = 0;
}

void Bienvenida(void){
    ConfiguraLCD(4);
    InicializaLCD();
    OcultarCursor();

    CrearCaracter(Estrella, 0);

    EscribeLCD_c(0);
    EscribeLCD_c(0);
    MensajeLCD_Var("   Bienvenido ");
    DireccionaLCD(0xC0);
    EscribeLCD_c(0);
    EscribeLCD_c(0);
    MensajeLCD_Var("    Usuario");
    __delay_ms(3200);

    for(int i = 0; i < 18; i++){
        DesplazaPantallaD();
        __delay_ms(100);
    }
}

void PreguntaAlUsuario(void){
    while(1){
        CrearCaracter(RayaAlPiso, 1);
        posicion=0;
        BorraLCD();
        MensajeLCD_Var("Piezas a contar:");
        DireccionaLCD(0xC7);
        EscribeLCD_c(1);
        EscribeLCD_c(1);
        DireccionaLCD(0xC7);
        MostrarCursor();

        editor=1;
        while(Tecla!= '*'){}

        if((Objetivo>59)||(Objetivo==0)){
            editor=0;
            Tecla='\0';
            Objetivo=0;
            BorraLCD();
            OcultarCursor();
            MensajeLCD_Var("     !Error!");
            __delay_ms(1000);
            BorraLCD();
            MensajeLCD_Var("Valor max: 59");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("Valor min: 01");
            __delay_ms(2000);
            BorraLCD();
        }else{
            editor=0;
            posicion='0';
            BorraLCD();
            break;
        }
    }
}

void ConfigPregunta(){
    if(posicion==0 && editor==1){
        EscribeLCD_n8(Tecla,1);
        Objetivo=Tecla;
    }else if(posicion==1 && editor==1){
        EscribeLCD_n8(Tecla,1);
        Objetivo=Objetivo*10+Tecla;
        OcultarCursor();
    }
    posicion++;
}

void Borrar(){
    if(editor==1){
        MostrarCursor();
        Objetivo=0;
        posicion=0;
        DireccionaLCD(0xC7);
        EscribeLCD_c(1);
        EscribeLCD_c(1);
        DireccionaLCD(0xC7);
    }
}

unsigned int Conversion(unsigned char canal){
    ADCON0=(canal<<2);
    ADON=1;
    GO_DONE=1;
    while(GO_DONE==1);
    return ADRES;
}

void putch(char data){
    while(TRMT==0);
    TXREG=data;
}

unsigned char MedirDistancia(void){
  unsigned char aux=0;

  CCP2CON=0b00000100;
  TMR1=0;
  TMR1IF=0;
  CCP2IF=0;

  TRIGGER=1;
  __delay_us(10);
  TRIGGER=0;

  etimeout=1;
  while(ECHO==0  && etimeout==1);
  if(etimeout==0){
    return 0;
  }

  TMR1ON=1;
  while(CCP2IF==0 && TMR1IF==0);
  TMR1ON=0;

  if(TMR1IF==1)
    aux=255;
  else{
   if(CCPR2>=3556)
      CCPR2=3556;
    aux=CCPR2/14;
  }

  return aux;
}

unsigned char EEPROM_Read(unsigned char address){
    EEADR = address;
    EEPGD = 0;
    CFGS = 0;
    RD = 1;
    return EEDATA;
}

void EEPROM_Write(unsigned char address, unsigned char data){
    EEADR = address;
    EEDATA = data;
    EEPGD = 0;
    CFGS = 0;
    WREN = 1;
    GIE = 0;
    EECON2 = 0b01010101;
    EECON2 = 0b10101010;
    WR = 1;
    GIE = 1;
    while(WR);
    WREN = 0;
}

void GuardarContador(unsigned int valor, unsigned int objetivo){
    EEPROM_Write(0b00000000, (unsigned char)(valor & 0xFF));
    EEPROM_Write(0b00000001, (unsigned char)(objetivo & 0xFF));
}

unsigned int LeerContador(void){
    return (unsigned int)EEPROM_Read(0b00000000);
}

unsigned int LeerObjetivo(void){
    return (unsigned int)EEPROM_Read(0b00000001);
}

void Motor_Avance(void){
    modoReversa = 0;
    LATA6 = 0;
}

void Motor_Reversa(void){
    modoReversa = 1;
    PWM_Set10bit(0);
    LATA6 = 1;
}

void Motor_Paro(void){
    modoReversa = 0;
    PWM_Set10bit(0);
    LATA6 = 0;
}
