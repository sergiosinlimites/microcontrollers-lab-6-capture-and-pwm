#include <xc.h>
#include <stdio.h>
#define _XTAL_FREQ 1000000 //Definir la constante para el cálculo de retardos  
#include "LibLCDXC8.h" //Incluir libreria propia

#pragma config FOSC=INTOSCIO_EC // <-- CAMBIO libera RA6 como I/O (antes INTOSC_EC)
#pragma config WDT=OFF //Desactivar el perro guardian
#pragma config LVP=OFF //Programar el PIC

#define TRIGGER RC0 //Declaración del pin para el Trigger 
#define ECHO RC1    //Declaración del pin para el ECHO

//DECLARACIÓN DE VARIABLES 
unsigned char Corazon[8] = {
    0b01010,  //  * * 
    0b11111,  // *****
    0b11111,  // *****
    0b11111,  // *****
    0b01110,  //  *** 
    0b00100,  //   *  
    0b00000,  //      
    0b00000
};

unsigned char RayaAlPiso[]={
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b11111}; // Carácter nuevo

unsigned int Supercontador; //Contador global de las piezas
unsigned int contador; //Contador unidades (siete segmentos)
unsigned int contadorRGB; //Contador decenas (led rgb)
unsigned char posicion; // Posicion del número ingresado por el usuario (unidades o decenas)
unsigned char editor; // Variable para habilitar la escritura en el lcd del usuario 
unsigned int Objetivo; // Valor meta de las piezas a contar 
unsigned char salir; // Variable para salir del ciclo de conteo
unsigned char Tecla; // Tecla presionada en el teclado
unsigned char Pulsado; //Variable para evitar conteo infinito
unsigned char Inactividad; //Variable para inactividad
unsigned int valorADC; // Variable que guarda el valor dado por el modulo ADC
unsigned int valorADC2; // Variable que guarda el valor dado por el modulo ADC
unsigned char datoSerial;
unsigned char d; // Variable que guarda el valor de la distancia 
unsigned char etimeout=0,ctimeout=0;
unsigned char porcentajeADC;
unsigned int anchoPulso=0;
unsigned int valorGuardado=0;
unsigned int objetivoGuardado=0;

// --- NUEVO: bandera para bloquear PWM en reversa ---
unsigned char modoReversa = 0; //0=avance(PWM activo), 1=reversa(PWM forzado a 0, RA6=1)

//DECLARACIÓN DE FUNCIONES 
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

void main (void){
    //CONFIGURACION DE LOS VALORES INICIALES DE LAS VARIABLES
    ConfigVariables();
    editor=0;
    datoSerial=0;

    // CONFIGURACIÓN DE LOS PUERTOS
    ADCON1=0b001110; //Quital las funciones analogas de los pines RA1-RA4, RB0-RB4 , RE0-RE2  
                     // solo el pin A0 es analógico

    //PWM: Se utiliza CCP1 (como está)
    TRISC2=0; //pin de salida para correcto funcionamiento
    PR2=249; //Frecuencia del pwm de 1 KHz      PR2=1ms*1M/1=250
    CCPR1L = 125; //Valor del ancho de pulso (Tiempo en alto) CCPXL=(250+1)*50%=125.5
    T2CON=0b0000000; //Valor de prescaler en 1 y se mantiene apagado 
    CCP1CON =0b00001100; // Modo PWM 
    TMR2=0;// Timer2 empieza en 0
    TMR2ON=1; //Se prende el tmr2

    // --- NUEVO: configuración reversa (RA6) ---
    TRISA6 = 0;
    LATA6  = 0;   // por defecto 0
    modoReversa  = 0;   // por defecto avance

    // CONFIGURACIÓN MÓDULO ADC
    ADCON0=0b00000001; // Se selecciona el canal 0 y se enciende el módulo
    ADCON2=0b10001000; // Se justifica hacia la derecha y se toma un preescaler de 2 

    //SENSOR ULTRASONICO
    TRISC0=0; //Se ajusta el pin de Trigger como salida
    TRISC1=1; //Pin C1 es configurado como entrada digital
    LATC0=0; // La salida del Pin C0 es 0
    T1CON=0b10010000;   //Ajuste de timer1: prescaler 2 T_sobreflujo=0.06s?

    // Pines para el RGB
    TRISE=0; // Todos los pines del puerto E son salidas digitales
    LATE=0b00000111; // Todos los pines de salida del puerto E en 0

    // Pines para el Siete segmentos
    TRISD=0; // Todos los pines del puerto E son salidas digitales
    LATD=contador; // El puerto es igual a el valor del contador 

    // Pin del led de operación
    TRISA1=0; // Pin A1 es configurado como salida digital
    LATA1=0; // La salida del Pin A1 es 0

    TRISA2=0; // Pin A2 es configurado como salida digital para el Buzzer
    LATA2=0; // La salida del Pin A2 es 0

    // Pines para el uso de la lcd
    TRISA3=0; //RS
    LATA3=0;
    TRISA4=0; //E

    //Pines transmisión serial
    TRISC6=0; //Tx
    TRISC7=1; //Rx
    TXSTA=0b00100100;//Configurar la transimision / modo asincronico-8 bits/
    RCSTA = 0b10010000; // Serial Port Enable 
    BAUDCON=0b00001000;//Velocidad de transimsion 9600
    SPBRG=25;//Ajustar baude rate

    //Pin del pulsador de conteo
    TRISC1=1; //Pin C1 es configurado como entrada digital

    // Pin prender apagar backlight LCD
    TRISA5=0; 
    LATA5=1;  

    // CONFIGURACIÓN DE LAS INTERRUPCIONES //
    T0CON=0b00000001; // timer0 modo 16 bits - prescale 4
    TMR0=3036; 
    TMR0IF=0; 
    TMR0IE=1; 
    TMR0ON=1;

    // Configuración de la interrupción del TIMER1  
    T1CON=0b10000000;   //prescaler 1

    //Configuración iterrupción teclado (pueto B)
    TRISB=0b11110000; 
    LATB=0b00000000;
    RBPU=0; 
    __delay_ms(100); 
    RBIF=0; 
    RBIE=1;

    PEIE=1; 
    GIE=1;  
    RCIE=1;

    //////////////////////////////////////////////////////////////////////////
    LATA3=1;
    Bienvenida(); 

    //Detectar fuente de Reset POR
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
                if(contadorRGB==0)      LATE=0b00000010;
                else if(contadorRGB==1) LATE=0b00000011;
                else if(contadorRGB==2) LATE=0b00000001;
                else if(contadorRGB==3) LATE=0b00000101;
                else if(contadorRGB==4) LATE=0b00000100;
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

        LATE=0b00000010;
        LATD=contador;
    }
}

void __interrupt()ISR(void){

    if (RCIF==1){
        Inactividad=0;
        datoSerial = RCREG; 

        if(datoSerial==80 || datoSerial==112){ // P/p
            LATE=0b00000110;
            BorraLCD(); 
            OcultarCursor();
            MensajeLCD_Var("   PARADA DE");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("   EMERGENCIA");

            CCPR1L=0;
            Motor_Paro();     // <-- NUEVO: asegura RA6=0 y PWM=0
            while(1){}
        }
        else if(salir==1 && (datoSerial==114 || datoSerial==82)){ // r/R
            contador=0;
            Supercontador = 0;
            contadorRGB = 0;
            GuardarContador(0, Objetivo);
            LATE=0b00000010; 
            DireccionaLCD(0x8B);
            EscribeLCD_n8(Objetivo-Supercontador,2);
            LATD=contador;
        }
        // --- NUEVO: control sentido por serial ---
        else if(datoSerial=='F' || datoSerial=='f'){ // Avance
            Motor_Avance();
        }
        else if(datoSerial=='G' || datoSerial=='g'){ // Reversa
            Motor_Reversa();
        }
    }

    if(TMR0IF==1){
        Inactividad++;
        TMR0=3036;
        TMR0IF=0;
        LATA1=LATA1^1;

        valorADC=Conversion(0);
        porcentajeADC=100*valorADC/1023;

        if(valorADC>valorADC2+3 && valorADC<valorADC2-3){
            Inactividad=0;
        }

        printf("Valor del ADC:%d\r\n", valorADC);

        if(valorADC>664)
            porcentajeADC=porcentajeADC+65;

        if(datoSerial==90 || datoSerial==122){
            CCPR1L = 0;
            porcentajeADC=0;
        }else if(datoSerial==88 || datoSerial==120){
            CCPR1L = 50;
            porcentajeADC=20;
        }else if(datoSerial==67 || datoSerial==99){
            CCPR1L = 100;
            porcentajeADC=40;
        }else if(datoSerial==86 || datoSerial==118){
            CCPR1L = 150;
            porcentajeADC=60;
        }else if(datoSerial==66 || datoSerial==98){
            CCPR1L = 200;
            porcentajeADC=80;
        }else if(datoSerial==78 || datoSerial==110){
            CCPR1L = 250;
            porcentajeADC=100;
        }
        else{
            CCPR1L = porcentajeADC*250/100;
        }

        // --- NUEVO: si está en reversa, FORZAR PWM a 0 ---
        if(modoReversa==1){
            CCPR1L = 0;
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

    // --- TECLADO: (igual que tu código, SIN cambios) ---
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
                    CCPR1L=0;
                    Motor_Paro(); // <-- NUEVO: RA6=0 y PWM=0
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
                            if(contadorRGB==0)      LATE=0b00000010;
                            else if(contadorRGB==1) LATE=0b00000011;
                            else if(contadorRGB==2) LATE=0b00000001;
                            else if(contadorRGB==3) LATE=0b00000101;
                            else if(contadorRGB==4) LATE=0b00000100;
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

    // --- NUEVO: defaults motor ---
    modoReversa = 0;
    LATA6 = 0;
}

void Bienvenida(void){
    ConfiguraLCD(4);
    InicializaLCD();
    OcultarCursor();

    CrearCaracter(Corazon, 0);

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
    EEPROM_Write(0b00000000, (unsigned char)(valor & 0b11111111));    
    EEPROM_Write(0b00000001, (unsigned char)(objetivo & 0b11111111));  
}

unsigned int LeerContador(void){
    return (unsigned int)EEPROM_Read(0b00000000);
}

unsigned int LeerObjetivo(void){
    return (unsigned int)EEPROM_Read(0b00000001);
}

// =====================
// NUEVO: Funciones motor
// =====================
void Motor_Avance(void){
    modoReversa = 0;
    LATA6 = 0;   // reversa apagada
    // PWM queda controlado por Timer0 como siempre
}

void Motor_Reversa(void){
    modoReversa = 1;
    CCPR1L = 0; // PWM a 0
    LATA6 = 1;  // reversa en 1 (L298)
}

void Motor_Paro(void){
    modoReversa = 0;
    CCPR1L = 0; // PWM a 0
    LATA6 = 0;  // reversa apagada
}
