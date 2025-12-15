#include <xc.h>
#include <stdio.h>
#define _XTAL_FREQ 1000000 //Definir la constante para el cálculo de retardos  
#include "LibLCDXC8_1.h" //Incluir libreria propia
#pragma config FOSC=INTOSC_EC //Configurar el reloj interno
#pragma config WDT=OFF //Desactivar el perro guardian
#pragma config LVP=OFF //Programar el PIC
#define TRIGGER RC0 //Declaración del pin para el Trigger 
#define ECHO RC1 //Declaración del pin para el ECHO

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

void main (void){
    //CONFIGURACION DE LOS VALORES INICIALES DE LAS VARIABLES
    ConfigVariables();
    editor=0;
    datoSerial=0;

    // CONFIGURACIÓN DE LOS PUERTOS
    ADCON1=0b001110; //Quital las funciones analogas de los pines RA1-RA4, RB0-RB4 , RE0-RE2  
                     // solo el pin A0 es analógico

    // Pin para la salida del motor
//    TRISC2=0;
//    LATC2=0;
    
    //PWM: Se utiliza CCP1 
    TRISC2=0; //pin de salida para correcto funcionamiento
    PR2=249; //Frecuencia del pwm de 1 KHz      PR2=1ms*1M/1=250
    CCPR1L = 125; //Valor del ancho de pulso (Tiempo en alto) CCPXL=(250+1)*50%=125.5
    T2CON=0b0000000; //Valor de prescaler en 1 y se mantiene apagado 
    CCP1CON =0b00001100; // Modo PWM 
    TMR2=0;// Timer2 empieza en 0
    TMR2ON=1; //Se prende el tmr2
    
    
    
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
    //  CSRC =0; Modo maestro/esclavo(Solo para modo sincrono); 1=Modo maestro(señal de reloj interno); 0=Modo esclavo(señal de reloj externo)
    //  TX9 =0; Sleccion modo 8/9 bits en la transmision; 1=Modo 9 bits en la transmision; 0=Modo 8 bits en la transmision
    //  TXEN=1; Habilitador de Transmisor; 1= Habilitado; 0 = deshabilitado
    //  SYNC=0; Seleccion del modo de trabajo USART; 1 = Modo Sincrono; 0 = Modo asincrono.
    //  SENDB=0; Seleccion de envio del Caracter break en modo asincrono; 1=Envia caracter break en la proxima transmision; 0=Envio del caracter de transmision break completo.////////////////
    //  BRGH=1; Seleccion del modo alta/baja velocidad; 1=Alta; 0=Baja
    //  TRMT=0; Estado del registro de transmision TXREG; 1=Registro TXREG vacio; 0=Registro TXREG ocupado
    //
    RCSTA = 0b10010000; // Serial Port Enable 
    //  SPEN =1; Habilitador del puerto serie; 1=Puerto serie activado; 0=Desactivado
    //  RX9 =0; Sleccion modo 8/9 bits en la recepcion; 1=Modo 9 bits; 0=Modo 8 bits
    //  SREN=0; Habilitador de recepcion simple(SOLO EN MODO SINCRONO MAESTRO); 1= Habilitado; 0 = deshabilitado
    //  CREN=1; Habilitacion del receptor; 1 = Recepcion Habilitada; 0 = Recepcion deshabilitada
    //  ADDEN=0; Habilitacion de la deteccion de la direccion(SOLO EN MODO ASINCRONO DE 9 BITS); 1=Habilitado; 0=Deshabilitado.
    //  FERR=0; Bit de error en el formato del byte recibidio; 1=Se ha producido error en el formato; 0=No se ha producido error en el formato
    //  OERR=0; Bit de error de sobre escritura; 1=Se ha producido error de sobre escritura; 0=No Se ha producido error de sobre escritura
    //  RX9D=0; 9vo bit de Dtos de recepcion(CUANDO SE OPERAN TRAMAS SERIALES DE 9 BITS DE DATOS)

    BAUDCON=0b00001000;//Velocidad de transimsion 9600
    //  ADBOVF =0; Bit de desbordamiento de Auto-detecciond del baud rate; 1=Se ha producido un auto-desbordamiento; 0=No
    //  RCIDL =0; Estado de la operacion de la recepcion; 1=No hay operacion de recepcion en marcha; 0=Si hay operacion de recepion en marcha
    //  --
    //  SCKP=0; Seleccion de polaridad del reloj(Solo en modo sincrono); 1 = Flanco de subida; 0 = Flanco de bajada
    //  BRG16=1; Seleccion de 8/16 bits de velocidad del Baud rate; 1=Velocidad de 16 bits(SPBRG Y SPBRGH); 0=Velocidad de 8 bits(SPBRG).
    //  --
    //  WUE=0; Habilitacion de autodeteccion de la trama(Solo en modo asincrono); 1=Activado(Cuando se detecta un flanco de bajada en el pin RX se pone a 1 en Flag RCIF); 0=Desactivado
    //  ABDEN=0; Habilitacion de autodeteccion de baud rate; 1=Activado en la siguiente recepcion; 0=Desactivado


    SPBRG=25;//Ajustar baude rate
    //SPBRG=((Fosc)/(4*BAUD))-1 = ((1MHz)/(4*9600))-1 = 25.041

    // BAUD=(Fosc)/(4*SPBRG + 1) = (1 MHz)/(4*25 + 1) = 9615.38

    //Pin del pulsador de conteo
    TRISC1=1; //Pin C1 es configurado como entrada digital

    // Pin prender apagar backlight LCD
    TRISA5=0; // Pin A1 es configurado como salida digital
    LATA5=1; // La salida del Pin A5 es 0 (backlight prendido)  

    // CONFIGURACIÓN DE LAS INTERRUPCIONES //

    // Configuración de la interrupción del TIMER0
    T0CON=0b00000001; //Configuración del timer0 modo 16 bits - prescale 4
    TMR0=3036; // Valor de precarga
    TMR0IF=0; // Bandera inicializada en 0
    TMR0IE=1; // Habilitación local de la interrupción 
    TMR0ON=1; // Encender el Timer0

    // Configuración de la interrupción del TIMER1  
    T1CON=0b10000000;   //Ajuste de timer1: prescaler 1

    //Configuración iterrupción teclado (pueto B)
    TRISB=0b11110000; // Configura de RB0 a RB3 como salidas y de RB4 a RB7 como entradas
    LATB=0b00000000;// Salidas del puerto B = 0
    RBPU=0; //Activa resistencias de pull-up para el puerto B
    __delay_ms(100); //Delay mientras se polarizan las entradas
    RBIF=0; // Bandera a cero
    RBIE=1; // Activación de la interrupción de teclado

    PEIE=1; // Habilitar interrupciones de perifericos
    GIE=1;  //Habilitación global de las interrupciones
    RCIE=1;

    //////////////////////////////////////////////////////////////////////////
    LATA3=1;
    Bienvenida(); //Mensaje de Bienvenida en el LCD

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
         
        // Preguntar si desea continuar
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
            while(Tecla != 1 && Tecla != 2){} // Esperar respuesta

            if(Tecla == 1){ // Usuario elige continuar
                Supercontador = valorGuardado;
                Objetivo = objetivoGuardado;
                contadorRGB = Supercontador / 10;
                contador = Supercontador - (contadorRGB * 10);

                // Actualizar displays
                LATD = contador;
                if(contadorRGB==0){
                    LATE=0b00000010; //Magenta
                }else if(contadorRGB==1){
                    LATE=0b00000011; // Azul
                }else if(contadorRGB==2){
                    LATE=0b00000001; // Cyan
                }else if(contadorRGB==3){
                    LATE=0b00000101; // Verde
                }else if(contadorRGB==4){
                    LATE=0b00000100; // Amarillo
                }else if(contadorRGB==5){
                    LATE=0b00000000; //Blanco
                }

                BorraLCD();
                MensajeLCD_Var("Cuenta");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("restaurada");
                __delay_ms(1500);
                BorraLCD();
                
                goto inicio_conteo;
                
            } else { // Usuario elige reiniciar
                ConfigVariables();
                EEPROM_Write(0b00000000, 0b00000000);
                EEPROM_Write(0b00000001, 0b00000000);
                BorraLCD();
                MensajeLCD_Var("Cuenta");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("reiniciada");
                
                //__delay_ms(10);
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
        //Mensaje en pantalla
        MensajeLCD_Var("Faltantes: ");
        EscribeLCD_n8(Objetivo-Supercontador,2);
        DireccionaLCD(0xC0);    
        MensajeLCD_Var("Objetivo: ");
        EscribeLCD_n8(Objetivo,2);
        salir=1;
        //
        while (salir==1){
            __delay_ms(100); // La medición de la distancia se hace cada 100 ms
            d=MedirDistancia();
            if(Supercontador==Objetivo){ //Verifica si se llega al objetivo

                LATA2=1;
                __delay_ms(1000);
                LATA2=0;

                // Mensaje en pantalla
                BorraLCD();
                MensajeLCD_Var("Cuenta Cumplida");
                DireccionaLCD(0xC0);
                MensajeLCD_Var("   Presione OK");
                //
                salir = 0; //Se hace la salida 0 para que se salga del ciclo de conteo
                Tecla='\0'; //valor vacio
                while(Tecla!= '*'){} // Espera de pulso ok
                ConfigVariables(); // Configuracion a valores iniciales

            }
            if((d>4 && d<9) && Supercontador!=Objetivo){                                // Verifica si el interruptor está pulsado
                Inactividad=0;                                                  // Si esta contnado no quiero que entre en sleep
                Pulsado=0;  
            }

            if(Pulsado==0){
                if((d<4 || d>9)){
                    Pulsado=1;
                    contador++; // Aumenta contador
                    Supercontador++; // Aumenta Supercontador
                    GuardarContador(Supercontador, Objetivo);
                    if (contador==10){ // Unidades

                        LATA2=1;
                        __delay_ms(300);
                        LATA2=0;

                        contador=0;
                        contadorRGB++; // Aumento de decenas 
                        if (contadorRGB==6){ // Decenas 

                            LATA2=1;
                            __delay_ms(300);
                            LATA2=0;

                            contadorRGB=0;
                        }
                    }                
                    //RGB (decenas)
                    if(contadorRGB==0){
                        LATE=0b00000010; //Magenta
                    }else if(contadorRGB==1){
                        LATE=0b00000011; // Azul
                    }else if(contadorRGB==2){
                        LATE=0b00000001; // Cyan
                    }else if(contadorRGB==3){
                        LATE=0b00000101; // Verde
                    }else if(contadorRGB==4){
                        LATE=0b00000100; // Amarillo
                    }else if(contadorRGB==5){
                        LATE=0b00000000; //Blanco
                    }
                    //Mensaje en pantalla
                    DireccionaLCD(0x8B);
                    EscribeLCD_n8(Objetivo-Supercontador,2);
                    //
                    LATD=contador; // Salida del siete segmentos 
                    __delay_ms(500);  // delay pa' el pulsador   
                }
            }   
        }

        LATE=0b00000010; // Rgb (Magenta) 
        LATD=contador; // Salida siete segmentos
    }
}

void __interrupt()ISR(void){

    if (RCIF==1){
        Inactividad=0;
        datoSerial = RCREG; 
        if(datoSerial==80 || datoSerial==112){
            LATE=0b00000110; //Led en rojo
            //Mensaje en pantalla
            BorraLCD(); 
            OcultarCursor();
            MensajeLCD_Var("   PARADA DE");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("   EMERGENCIA");
            //
            CCPR1L=0;
            while(1){} // Bucle
        }else if(salir==1 && (datoSerial==114 || datoSerial==82)){
            contador=0;
            Supercontador = 0;
            contadorRGB = 0;
            GuardarContador(0, Objetivo);
            LATE=0b00000010; // Rgb (Magenta) 
            DireccionaLCD(0x8B);
            EscribeLCD_n8(Objetivo-Supercontador,2); //Mensaje pantalla
            LATD=contador; //Salida del siete segmentos

        }

        if(datoSerial==90 || datoSerial==122){                                             //Z o z
            CCPR1L = 0; //Valor del ancho de pulso del PWM
            porcentajeADC=0;
        }else if(datoSerial==88 || datoSerial==120){                                       //X o x
            CCPR1L = 50; //Valor del ancho de pulso del PWM
            porcentajeADC=20;
        }else if(datoSerial==67 || datoSerial==99){                                        //C o c
            CCPR1L = 100; //Valor del ancho de pulso del PWM
            porcentajeADC=40;
        }else if(datoSerial==86 || datoSerial==118){                                       //V o v
            CCPR1L = 150; //Valor del ancho de pulso del PWM
            porcentajeADC=60;
        }else if(datoSerial==66 || datoSerial==98){                                       //B o b
            CCPR1L = 200; //Valor del ancho de pulso del PWM
            porcentajeADC=80;
        }else if(datoSerial==78 || datoSerial==110){                                       //N o n
            CCPR1L = 250; //Valor del ancho de pulso del PWM
            porcentajeADC=100;
        }

    }

    if(TMR0IF==1){ // Led de operación  
        Inactividad++;   // Contar segundos
        TMR0=3036; //Valor de precarga
        TMR0IF=0; //Bandera en 0
        LATA1=LATA1^1; // Prende o apaga el led 
        valorADC=Conversion(0); // Se consuta el valor del conversor
        porcentajeADC=100*valorADC/1023; //conversion a porcentaje
        if(valorADC>valorADC2+3 && valorADC<valorADC2-3){
            Inactividad=0;
        }
        printf("Valor del ADC:%d\r\n", valorADC); // Se envia de manera serial
        
        if(valorADC>664)// Se desborda la memoria
            porcentajeADC=porcentajeADC+65;
        
        if(datoSerial==90 || datoSerial==122){                                             //Z o z
            CCPR1L = 0; //Valor del ancho de pulso del PWM
            porcentajeADC=0;
        }else if(datoSerial==88 || datoSerial==120){                                       //X o x
            CCPR1L = 50; //Valor del ancho de pulso del PWM
            porcentajeADC=20;
            
        }else if(datoSerial==67 || datoSerial==99){                                        //C o c
            CCPR1L = 100; //Valor del ancho de pulso del PWM
            porcentajeADC=40;
            
        }else if(datoSerial==86 || datoSerial==118){                                       //V o v
            CCPR1L = 150; //Valor del ancho de pulso del PWM
            porcentajeADC=60;
            
        }else if(datoSerial==66 || datoSerial==98){                                       //B o b
            CCPR1L = 200; //Valor del ancho de pulso del PWM
            porcentajeADC=80;
            
        }else if(datoSerial==78 || datoSerial==110){                                       //N o n
            CCPR1L = 250; //Valor del ancho de pulso del PWM
            porcentajeADC=100;
            
        }
        else CCPR1L = porcentajeADC*250/100; //Valor del ancho de pulso del PWM
        
         
        
        printf("Valor de PWM: %d%%\r\n",porcentajeADC); // Se envia de manera serial (mirar putch)
        
        if (etimeout == 1)  // Si etimeout está activado (igual a 1)
            ctimeout++;     // Incrementar ctimeout en 1 (se cuenta cuántas veces ha pasado el tiempo)
        else
            ctimeout = 0;   // Si etimeout está desactivado (igual a 0), reiniciar ctimeout a 0

        if (ctimeout >= 2)  // Si ctimeout ha alcanzado un valor de 2 (aproximadamente 2 segundos)
            etimeout = 0;   // Desactivar etimeout, terminando así el modo antibloqueo 
        
        if(d==0)//Si la medida es 0 es por error del sensor
            printf("Falla en el sensor\r\n");
        
        else//Si no hay error se transmite la distancia
            printf("La distancia medida es: %d cm\r\n",d);
        
        
        // Apagar backlight a los 10 s
        if(Inactividad == 10){
            //Backlight = 0;
            LATA3 = 0;
        }

        // Entrar en suspensión a los 20 s
        if(Inactividad >= 20){
            Sleep();    // suspender PIC

            // ---- DESPERTÓ AQUI ----
            Inactividad = 0;             // reiniciar inactividad
            RBIF = 0;                    // limpiar interrupción por teclado
            TMR1ON = 1;                  // volver a encender Timer1
        }
        valorADC2=valorADC;
    }


    if(RBIF==1){
        if(PORTB!=0b11110000){   
            Inactividad = 0;                                                    // Hubo actividad
            LATB=0b11111110;
            if(RB4==0){                                                         //1
                Tecla=1; 
                ConfigPregunta();
            }            
            else if(RB5==0) {                                                   //2
                Tecla=2; 
                ConfigPregunta();

            }
            else if(RB6==0){                                                    //3
                Tecla=3; 
                ConfigPregunta();

            }
            else if(RB7==0){                                                    //OK
                Tecla='*'; 
            }




            else{
                LATB=0b11111101;
                if(RB4==0){                                                     //4
                Tecla=4;
                ConfigPregunta();
                }
                else if(RB5==0) {                                               //5
                    Tecla=5; 
                    ConfigPregunta();
                }
                else if(RB6==0) {                                               //6
                    Tecla=6; 
                    ConfigPregunta();
                }
                else if(RB7==0) {                                               //PARADA EMERGENCIA
                    LATE=0b00000110; //Led en rojo
                    //Mensaje en pantalla
                    BorraLCD(); 
                    OcultarCursor();
                    MensajeLCD_Var("   PARADA DE");
                    DireccionaLCD(0xC0);
                    MensajeLCD_Var("   EMERGENCIA");
                    //
                    CCPR1L=0;
                    while(1){} // Bucle
                }   


            else{
                LATB=0b11111011;
                if(RB4==0) {                                                    //1
                    Tecla=7; 
                    ConfigPregunta();
                }
                else if(RB5==0) {                                               //2
                    Tecla=8; 
                    ConfigPregunta();
                }
                else if(RB6==0) {                                               //3
                    Tecla=9; 
                    ConfigPregunta();
                } 
                else if(RB7==0) {                                               //SUPR
                    Borrar();
                }



            else{
                LATB=0b11110111;                                                //REINICIO
                if(RB4==0){             
                    contador=0;
                    Supercontador = 0;
                    contadorRGB = 0;
                    GuardarContador(0, Objetivo); 
                    LATE=0b00000010; // Rgb (Magenta) 
                    if(salir==1){
                        DireccionaLCD(0x8B);
                        EscribeLCD_n8(Objetivo-Supercontador,2); //Mensaje pantalla
                        LATD=contador; //Salida del siete segmentos
                    }
                }
                else if(RB5==0) {                                               //0
                    Tecla=0; 
                    ConfigPregunta();
                }
                else if(RB6==0) {                                               //FIN
                    Borrar();
                    Supercontador=Objetivo;
                    contadorRGB = Objetivo/10;
                    contador = Objetivo-contadorRGB*10;
                    if(contadorRGB==0){
                        LATE=0b00000010; //Magenta
                    }else if(contadorRGB==1){
                        LATE=0b00000011; // Azul
                    }else if(contadorRGB==2){
                        LATE=0b00000001; // Cyan
                    }else if(contadorRGB==3){
                        LATE=0b00000101; // Verde
                    }else if(contadorRGB==4){
                        LATE=0b00000100; // Amarillo
                    }else if(contadorRGB==5){
                        LATE=0b00000000; //Blanco
                    }
                    LATD=contador; // Salida del siete segmentos
                }
                else if(RB7==0) {                                           //LUZ
                    LATA3=LATA3^1;
                    TMR1ON=1; // Encender Timer1
                }
            }
            }
            }
            LATB=0b11110000; // configuración default
        }
        __delay_ms(300); // delay pa' el pulsador 
        RBIF=0; // Bandera en 0
    }
}
void ConfigVariables(void){ //Valores iniciales de la variables
    Pulsado=1;
    valorADC2=0;
    contador=0; 
    salir=0;
    Supercontador=0;
    contadorRGB=0;
    posicion=0;
    Objetivo=0;
    Tecla='\0'; 
}
void Bienvenida(void){
    // CONFIGURACIÓN DEL LCD
    ConfiguraLCD(4); //Modo de bits  
    InicializaLCD(); //Inicialización de la pantalla
    OcultarCursor();

    //CREACIÓN DE CARACTERES PROPIOS
    CrearCaracter(Corazon, 0);

    //Mensaje en pantalla
    EscribeLCD_c(0);
    EscribeLCD_c(0);
    MensajeLCD_Var("   Bienvenido ");
    DireccionaLCD(0xC0);
    EscribeLCD_c(0);
    EscribeLCD_c(0);
    MensajeLCD_Var("    Usuario");
    __delay_ms(3200);

    // ? Mover a la derecha (aprox 2 s)
    for(int i = 0; i < 18; i++){
        DesplazaPantallaD();  // Shift right
        __delay_ms(100);
    }

    // Mensaje total < 5 segundos
}
void PreguntaAlUsuario(void){ //encargada del setup de la pregunta de elementos a contar
    while(1){
        CrearCaracter(RayaAlPiso, 1);
        posicion=0; //posicion del cursor en unidades
        //Mensaje en pantalla
        BorraLCD();
        MensajeLCD_Var("Piezas a contar:");
        DireccionaLCD(0xC7);
        EscribeLCD_c(1);
        EscribeLCD_c(1);
        DireccionaLCD(0xC7);
        MostrarCursor();
        //
        editor=1; // El usuario puede escribir en el lcd 
        while(Tecla!= '*'){ // no hacer nada mientras no se de al ok  
        // el ingreso del valor a contar se hace por medio de interrupciones
        }
        if((Objetivo>59)||(Objetivo==0)){
            editor=0; // El usuario no puede escribir en el lcd
            Tecla='\0';
            Objetivo=0;
            //Mensaje en pantalla
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
            //
        }else{
        editor=0; // El usuario no puede escribir en el lcd
        posicion='0';
        BorraLCD();
        break;
            }
    }
}
void ConfigPregunta(){ //Funcion para el ingreso del valor a contar
    if(posicion==0 && editor==1){
        EscribeLCD_n8(Tecla,1); //Mensaje en pantalla
        Objetivo=Tecla;
    }else if(posicion==1 && editor==1){
        EscribeLCD_n8(Tecla,1); //Mensaje en pantalla
        Objetivo=Objetivo*10+Tecla;
        OcultarCursor();
    }
    posicion++; 
}
void Borrar(){ //Borrar el valor escrito por el usuario 
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
    ADON=1;      //Se habilita el modulo ADC
    GO_DONE=1;   //Se inicia la conversión
    while(GO_DONE==1);
    return ADRES;
}
void putch(char data){ //Permite utilizar printf para la comunicacion serial
    while(TRMT==0);
    TXREG=data;
}
unsigned char MedirDistancia(void){
  unsigned char aux=0;
  CCP2CON=0b00000100; //Ajustar CCP en modo captura con flanco de bajada
  TMR1=0;             //Iniciamos el timer1 en 0
  TMR1IF=0;           // Limpiar bandera de overflow del Timer1
  CCP2IF=0;           //Iniciar bandera CCPx en 0
  TRIGGER=1;          //Dar inicio al sensor
  __delay_us(10);
  TRIGGER=0;
  etimeout=1;         //Se habilita la condición de antibloqueo
  while(ECHO==0  && etimeout==1); //Se espera que el sensor empiece la
                      //medición o que pase el antibloqueo (aprox 2s)
  if(etimeout==0){    //Si el sensor no responde se retorna un 0
    return 0;
  }   
  //ECHO==1
  TMR1ON=1;           //Se da inicio al timer1 o medición de tiempo
  while(CCP2IF==0 && TMR1IF==0);   //Espera a que la señal de ultrasonido regrese
  TMR1ON=0;           //Se da parada al timer 1 o medición de tiempo
  if(TMR1IF==1)       //Se comprueba que la medición del pulso del sensor no
    aux=255;          //exceda el rango del timer1, si es asi se limita a 255
  else{  
   if(CCPR2>=3556)  //Si el sensor excede 254cm se limita a este valor
      CCPR2=3556;
    aux=CCPR2/14; //Se calcula el valor de distancia a partir del tiempo donde 14 es aproximadamente a 58,7 / 4
  }
  return aux;         //Se retorna la medición de distancia obtenida
}

unsigned char EEPROM_Read(unsigned char address){
    EEADR = address;           // Cargar dirección
    EEPGD = 0;      // Acceso a EEPROM (no Flash)
    CFGS = 0;       // Acceso a EEPROM/Flash (no registros de config)
    RD = 1;         // Iniciar lectura
    return EEDATA;             // Retornar dato leído
}

void EEPROM_Write(unsigned char address, unsigned char data){
    EEADR = address;           // Cargar dirección
    EEDATA = data;             // Cargar dato a escribir
    EEPGD = 0;      // Acceso a EEPROM
    CFGS = 0;       // Acceso a EEPROM/Flash
    WREN = 1;       // Habilitar escritura
    
    // Secuencia obligatoria para escribir en EEPROM
    GIE = 0;        // Deshabilitar interrupciones
    EECON2 = 0b01010101;       // Primera clave (0x55)
    EECON2 = 0b10101010;       // Segunda clave (0xAA)
    WR = 1;         // Iniciar escritura
    GIE = 1;        // Rehabilitar interrupciones
    
    // Esperar a que termine la escritura
    while(WR);      
    WREN = 0;       // Deshabilitar escritura
}

// Guardar contador Y objetivo en EEPROM (usa 4 bytes)
void GuardarContador(unsigned int valor, unsigned int objetivo){
    EEPROM_Write(0b00000000, (unsigned char)(valor & 0b11111111));    
    EEPROM_Write(0b00000001, (unsigned char)(objetivo & 0b11111111));  
}
// Leer contador desde EEPROM
// Leer contador desde EEPROM
unsigned int LeerContador(void){
    return (unsigned int)EEPROM_Read(0b00000000);  // Leer solo 1 byte
}

// Leer objetivo desde EEPROM 
unsigned int LeerObjetivo(void){
    return (unsigned int)EEPROM_Read(0b00000001);  // Leer solo 1 byte
}
