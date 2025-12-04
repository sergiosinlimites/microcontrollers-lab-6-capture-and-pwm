#include <xc.h>
#include <stdio.h>
#include <stdlib.h> // Necesario para funciones estandar
#define _XTAL_FREQ 1000000 //Definir la constante para el cálculo de retardos   
#include "LibLCDXC8.h" //Incluir libreria propia
#pragma config FOSC=INTOSC_EC //Configurar el reloj interno
#pragma config WDT=OFF //Desactivar el perro guardian
#pragma config LVP=OFF //Programar el PIC

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
    0b11111
}; // Carácter nuevo

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
unsigned char datoSerial;

volatile unsigned char flag_imprimir = 0; 
unsigned int tiempo_eco;    // Duración del pulso Echo
float distancia;            // Distancia calculada
unsigned char pwm_duty_perc; // Porcentaje de PWM
unsigned char control_manual_pwm = 0; // Bandera para control serial (Bonus 1)
unsigned char conteo_recuperado = 0;  // Variable temporal EEPROM (Bonus 2)

//DECLARACIÓN DE FUNCIONES 
void __interrupt()ISR(void);
void ConfigVariables(void);
void Bienvenida(void);
void PreguntaAlUsuario(void);
void ConfigPregunta(void);
void Borrar(void);
unsigned int Conversion(unsigned char);
void putch(char);

void ConfigurarPWM(void);
void ConfigurarCaptura(void);
void TriggerUltrasonido(void);
void GuardarEEPROM(unsigned int direccion, unsigned char dato);
unsigned char LeerEEPROM(unsigned int direccion);


void main (void){

    //CONFIGURACION DE LOS VALORES INICIALES DE LAS VARIABLES
    ConfigVariables();
    editor=0;
    datoSerial=0;

    // CONFIGURACIÓN DE LOS PUERTOS
    ADCON1=0b001110; //Quital las funciones analogas de los pines RA1-RA4, RB0-RB4 , RE0-RE2  
                     // solo el pin A0 es analógico

    // CONFIGURACIÓN MÓDULO ADC
    ADCON0=0b00000001; // Se selecciona el canal 0 y se enciende el módulo
    ADCON2=0b10001000; // Se justifica hacia la derecha y se toma un preescaler de 2 


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

    // Pin para la salida del motor
    TRISC2=0; // RC2 como salida para PWM
    LATC2=0;
    
    // Pines Sensor Ultrasonido
    TRISC0=0; // RC0 como SALIDA para TRIGGER
    LATC0=0;
    TRISC1=1; // RC1 (CCP2) como ENTRADA para ECHO

    //Pines transmisión serial
    TRISC6=0; //Tx
    TRISC7=1; //Rx
    
    TXSTA=0b00100100;//Configurar la transimision / modo asincronico-8 bits/
        //  CSRC =0; Modo maestro/esclavo(Solo para modo sincrono); 1=Modo maestro(se al de reloj interno); 0=Modo esclavo(se al de reloj externo)
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

    ConfigurarPWM();        // Configura Motor
    ConfigurarCaptura();    // Configura Sensor

    // CONFIGURACIÓN DE LAS INTERRUPCIONES //

    // Configuración de la interrupción del TIMER0
    T0CON=0b00000001; //Configuración del timer0 modo 16 bits - prescale 4
    TMR0=3036; // Valor de precarga (aprox 1 seg)
    TMR0IF=0; // Bandera inicializada en 0
    TMR0IE=1; // Habilitación local de la interrupción 
    TMR0ON=1; // Encender el Timer0

    // Configuración de la interrupción del TIMER1  
    T1CON = 0b10110001;               // RD16, 1:8, interno, ON [web:16]
    TMR1=3036;
    TMR1IF=0; // Limpiar bandera de Timer1?
    TMR1IE=1; // Habilitacion local de la interrupcion de Timer1?
    TMR1ON = 1;

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
    // MODIFICADO GUIA 6 (Bonus 2): Recuperación de EEPROM al inicio
    // Detectar fuente de Reset POR y preguntar por recuperación
    if(POR==0){
        POR=1;
        conteo_recuperado = LeerEEPROM(0); // Leemos la dirección 0
        
        // Si hay un dato válido (mayor a 0 y menor a 60), preguntamos
        if(conteo_recuperado > 0 && conteo_recuperado < 60){
            Bienvenida();
            BorraLCD();
            MensajeLCD_Var("Recuperar conteo?");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("1:Si  2:No");
            
            Tecla = '\0';
            while(Tecla != 1 && Tecla != 2); // Esperar respuesta del teclado (1 o 2)
            
            if(Tecla == 1){
                // Restaurar
                Supercontador = conteo_recuperado;
                contadorRGB = Supercontador / 10;
                contador = Supercontador % 10;
                // Actualizar visualización
                LATD = contador;
                // Lógica rápida para restaurar color
                if(contadorRGB==0) LATE=0b00000010; 
                else if(contadorRGB==1) LATE=0b00000011; 
                else if(contadorRGB==2) LATE=0b00000001; 
                else if(contadorRGB==3) LATE=0b00000101; 
                else if(contadorRGB==4) LATE=0b00000100; 
                else if(contadorRGB==5) LATE=0b00000000;
            } else {
                Supercontador = 0;
                GuardarEEPROM(0, 0); // Borrar memoria
            }
        } else {
            // Comportamiento original si no hay dato válido
            Bienvenida(); 
            BorraLCD(); OcultarCursor();
            MensajeLCD_Var("    FALLA DE");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("     ENERGIA");
            __delay_ms(1000);
        }

    }else{
        Bienvenida(); 
        BorraLCD(); 
        OcultarCursor();
        MensajeLCD_Var("    RESET DE");
        DireccionaLCD(0xC0);
        MensajeLCD_Var("     USUARIO");
        __delay_ms(1000);
    }

    LATA3=0;

    while(1){

        
        if(flag_imprimir == 1){
            flag_imprimir = 0; // Bajamos la bandera
            
            // Aquí sí podemos tomarnos el tiempo de enviar datos
            printf("\r\nValor de PWM: %d %%", pwm_duty_perc);
            // Imprimimos como entero para evitar basura flotante en algunas versiones de printf
            printf("\r\nDistancia objeto: %d cm", (int)distancia);
        }
        
        if(Supercontador == 0) PreguntaAlUsuario(); // Solo preguntar si no se restauró o es 0
        OcultarCursor();
        //Mensaje en pantalla
        BorraLCD(); // Asegurar limpieza
        MensajeLCD_Var("Faltantes: ");
        EscribeLCD_n8(Objetivo-Supercontador,2);
        DireccionaLCD(0xC0);    
        MensajeLCD_Var("Objetivo: ");
        EscribeLCD_n8(Objetivo,2);
        salir=1;
        //
        while (salir==1){
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
                
                GuardarEEPROM(0, 0); // Limpiar EEPROM al terminar
                
                Tecla='\0'; //valor vacio
                while(Tecla!= '*'){} // Espera de pulso ok
                ConfigVariables(); // Configuracion a valores iniciales

            }
            
            // MODIFICADO GUIA 6: Lógica de sensor ultrasónico
            // Se usa la variable 'distancia' calculada en interrupción CCP2
            // Rango de detección según guía: 5 a 8 cm
            
            if(distancia >= 5.0 && distancia <= 8.0){
                Inactividad=0;
                if(Pulsado==0){ // Flanco de entrada
                    Pulsado=1;
                    contador++; // Aumenta contador
                    Supercontador++; // Aumenta Supercontador
                    
                    GuardarEEPROM(0, (unsigned char)Supercontador);

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
                    __delay_ms(500);  // delay para evitar rebote o conteo múltiple   
                }
            } else {
                // Objeto salió del rango o está lejos/muy cerca, permitir contar de nuevo
                if(distancia > 10.0 || distancia < 3.0) {
                    Pulsado = 0;
                }
            }   
        }

        LATE=0b00000010; // Rgb (Magenta) 
        LATD=contador; // Salida siete segmentos
    }
}

void ConfigurarPWM(void){
    // Configura Timer2 y CCP1 para PWM (Motor)
    // Frecuencia aprox 1kHz con 1MHz Fosc
    // Periodo PWM = [(PR2) + 1] * 4 * TOSC * (TMR2 Prescale Value=1)
    PR2 = 249; 
    CCP1CON = 0b00001100; // Modo PWM
    // 7-6: If CCP1M3:CCP1M2 = 11: 00 = Single output: P1A modulated; P1B, P1C, P1D assigned as port pins
    // 5-4: These bits are the two LSbs of the 10-bit PWM duty cycle. The eight MSbs of the duty cycle are found in CCPR1L.
    // 3-0: 1100 = PWM mode: P1A, P1C active-high; P1B, P1D active-high
    T2CON = 0b00000100;   // Timer2 ON
    // 7: unimplemented
    // 6-3: 0000 = 1:1 postscale
    // 2: timer2 ON
    // 1-0: 00 = Prescaler is 1

}

void ConfigurarCaptura(void){
    // Configura Timer3 y CCP2 para Captura (Sensor HC-SR04)
    T3CON = 0b10001001;
    // 7: RD16 = 1 Enables register read/write of Timer3 in one 16-bit operation
    // 6,3: T3CCP2:T3CCP1 = 0,1 Timer3 is the capture/compare clock source for CCP2;
    // 5-4: T3CKPS1:T3CKPS0 = 00 Prescaler 1:1
    // 2: ignorar
    // 1: 0 = Internal clock (FOSC/4)
    // 0: 1 = Enable Timer3
    CCP2CON = 0b00000101; // Modo Captura, flanco subida (Rising Edge)
    // 7-6: 00: Unimplemented
    // 5-4: 00 unused para captura
    // 3-0: 0101 = Capture mode, every rising edge
    CCP2IE = 1; // Habilitar interrupción CCP2, cuando llegue un dato del sensor
    CCP2IF = 0; // Bandera en 0
}

void TriggerUltrasonido(void){
    // Genera pulso de 10us en RC0
    LATC0 = 1;
    __delay_us(10);
    LATC0 = 0;
}

// Funciones para EEPROM
void GuardarEEPROM(unsigned int direccion, unsigned char dato){
    // WRERR es cero cuando termina la operación de escritura, 1 mientras escribe
    
    EEADR = direccion;
    EEDATA = dato;
    EECON1 = 0b0010;
    // 7: 0 = Access data EEPROM memory
    // 6: 0 = Access Flash program or data EEPROM memory
    // 5: 0 = Unimplemented
    // 4: 0 = Perform write-only
    // 3: 0 = The write operation completed. Nota: When a WRERR occurs, the EEPGD and CFGS bits are not cleared. This allows tracing of the error condition.
    // 2: 1 = Habilitar escritura
    // 1: Read only
    // 1 = Initiates a data EEPROM erase/write cycle or a program memory erase cycle or write cycle (The operation is self-timed and the bit is cleared by hardware once write is complete. The WR bit can only be set (not cleared) in software.)
    // 0 = Write cycle to the EEPROM is complete
    // 0: Read control bit
    // 1 = Initiates an EEPROM read (Read takes one cycle. RD is cleared in hardware. The RD bit can only be set (not cleared) in software. RD bit cannot be set when EEPGD = 1 or CFGS = 1.) 
    // 0 = Does not initiate an EEPROM read
    //EEPGD = 0; // Acceso a memoria de datos
    //EECON1bits.CFGS = 0;
    GIE = 0;   // Deshabilitar interrupciones
    EECON2 = 0x55; // Secuencia obligatoria
    EECON2 = 0xAA;
    EECON1bits.WR = 1;    // Iniciar escritura
    while(EECON1bits.WR); // Esperar a que termine, aprox 2ms
    GIE = 1;   // Habilitar interrupciones de nuevo
    EECON1bits.WREN = 0;  // Deshabilitar escritura
}

unsigned char LeerEEPROM(unsigned int direccion){
    EEADR = direccion;
    EECON1bits.EEPGD = 0; // 0=Acceso a memoria de datos
    EECON1bits.CFGS = 0; // 0=Acceso a memoria EEPROM, 1=Acceso a Registros de Configuración
    EECON1bits.RD = 1;    // Iniciar lectura
    return EEDATA;
}

void __interrupt()ISR(void){

    if (RCIF==1){
        datoSerial = RCREG; 
        
        // Control de velocidad manual (Bonus 1)
        if(datoSerial == 'Z' || datoSerial == 'z'){ control_manual_pwm = 1; pwm_duty_perc = 0; }
        else if(datoSerial == 'X' || datoSerial == 'x'){ control_manual_pwm = 1; pwm_duty_perc = 20; }
        else if(datoSerial == 'C' || datoSerial == 'c'){ control_manual_pwm = 1; pwm_duty_perc = 40; }
        else if(datoSerial == 'V' || datoSerial == 'v'){ control_manual_pwm = 1; pwm_duty_perc = 60; }
        else if(datoSerial == 'B' || datoSerial == 'b'){ control_manual_pwm = 1; pwm_duty_perc = 80; }
        else if(datoSerial == 'N' || datoSerial == 'n'){ control_manual_pwm = 1; pwm_duty_perc = 100; }
        else if(datoSerial == 'M' || datoSerial == 'm'){ control_manual_pwm = 0; } // Volver a control pot (opcional)

        // Comandos originales
        if(datoSerial==80 || datoSerial==112){ // P
            LATE=0b00000110; //Led en rojo
            //Mensaje en pantalla
            BorraLCD(); 
            OcultarCursor();
            MensajeLCD_Var("   PARADA DE");
            DireccionaLCD(0xC0);
            MensajeLCD_Var("   EMERGENCIA");
            CCP1CON = 0; // Apagar Motor PWM
            //
            while(1){} // Bucle
        }else if(salir==1 && (datoSerial==114 || datoSerial==82)){ // R
            contador=0;
            Supercontador = 0;
            contadorRGB = 0;
            LATE=0b00000010; // Rgb (Magenta) 
            DireccionaLCD(0x8B);
            EscribeLCD_n8(Objetivo-Supercontador,2); //Mensaje pantalla
            LATD=contador; //Salida del siete segmentos
            GuardarEEPROM(0,0); // Reiniciar el conteo y guardar en EEPROM
        }
    }

    // Interrupción Sensor HC-SR04
    if(CCP2IF){
        if(CCP2M0 == 1){ // Si estaba en flanco subida (Inicio Eco)
            // Reiniciar Timer3
            TMR3H = 0;
            TMR3L = 0;
            CCP2CON = 0b00000100; // Cambiar a flanco bajada (0100)
        } else { // Si detectó flanco bajada (Fin Eco)
            tiempo_eco = TMR3; // Leer valor 16 bits
            CCP2CON = 0b00000101; // Volver a esperar subida
            // Calculo: (tiempo_us / 58) = cm. Timer3 a 1MHz -> 1 tick = 1us
            distancia = (float)tiempo_eco / 58.0;
        }
        CCP2IF = 0; // Limpiar bandera
    }

    if(TMR0IF==1){ // Led de operación y lectura ADC/Serial
        TMR0=3036; //Valor de precarga
        TMR0IF=0; //Bandera en 0
        LATA1=LATA1^1; // Prende o apaga el led 
        
        TriggerUltrasonido();

        // Lógica PWM Motor y Serial
        if(control_manual_pwm == 0){
            valorADC=Conversion(0); 
            // Convertir 10-bit ADC a 10-bit PWM Duty (CCPR1L + DC1B)
            CCPR1L = valorADC >> 2; 
            DC1B1 = (valorADC & 2) >> 1;
            DC1B0 = valorADC & 1;
            // Calcular porcentaje para mostrar
            pwm_duty_perc = (unsigned char)((valorADC * 100UL) / 1023UL);
        } else {
            // Modo Manual por Serial
            unsigned int val_reg = (pwm_duty_perc * 1023UL) / 100UL;
            CCPR1L = val_reg >> 2;
            DC1B1 = (val_reg & 2) >> 1;
            DC1B0 = val_reg & 1;
        }
        flag_imprimir = 1;
    }

    if (TMR1IF){
        TMR1 = 34286;    // Recargar
        TMR1IF = 0;

        Inactividad++;   // Contar segundos

        // Apagar backlight a los 10 s
        if(Inactividad == 10){
            //Backlight = 0;
            LATA3 = 0;
        }

        // Entrar en suspensión a los 20 s
        if(Inactividad >= 20){
            CCP1CON = 0; // Apagar PWM antes de dormir
            Sleep();    // suspender PIC

            // ---- DESPERTÓ AQUI ----
            ConfigurarPWM(); // Restaurar PWM al despertar
            Inactividad = 0;             // reiniciar inactividad
            RBIF = 0;                    // limpiar interrupción por teclado
            TMR1ON = 1;                  // volver a encender Timer1
        }
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
                    CCP1CON = 0; // Apagar motor PWM
                    //
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
                    LATE=0b00000010; // Rgb (Magenta) 
                    if(salir==1){
                        DireccionaLCD(0x8B);
                        EscribeLCD_n8(Objetivo-Supercontador,2); //Mensaje pantalla
                        LATD=contador; //Salida del siete segmentos
                        GuardarEEPROM(0,0);
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
    __delay_ms(4200);

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