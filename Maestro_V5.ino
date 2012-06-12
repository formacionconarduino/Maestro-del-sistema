//---------------------------------------------------------------------------------------------------------------------------
//           Maestro domótica V 0.1.0.0
//           Autor: Juan Vila. 28/04/2012
//           Compilado con Arduino 1.0    
//           HW: Winkhel WK0500 + RTC
//           Librería Mudbus modificada. Corregida la respuesta a la función "Preset Multiple Registers".
//           Librería Mudbus permite escribir fuera del rango de Holding Registers (escribe en RAM). Pendiente de solucionar
//           Librería Mudbus configurada con 0 coils, 124 holding registers y 124 input registers
//           Librería SimpleModbusMaster Smm_Juan.h
//---------------------------------------------------------------------------------------------------------------------------

#define debug             // Abre terminal serie por puerto USB a 9600 8N1
#define debugRTU          // Depuración de comunicación RTU con los remotos
#define debugTCP          // Depuración de comunicación con los HMI en modbus TCP
//#define debugAltas      // Depuración para altas de nuevos elementos
#define debugComandosRTU  // Depuración para comandos recibidos por modbus TCP
#define debugRespuestas   // Copia en el terminal las respuestas a los comandos

#include <SPI.h>
#include <Ethernet.h>
#include "Mudbus.h"           // Servidor Modbus TCP
#include <Smm_Juan.h>         // Maestro Modbus RTU
#include <Wire.h>             // I2C (para RTC)
#include <EEPROM.h>
#include <Rtc_Pcf8563.h>      // RTC
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "comandos_tcp.h"    // Lista de comandos en modbus TCP
#include "elementos.h"       // Tipos de elementos domóticos válidos
#include "estructuras.h"     // Definición de estructuras para los tipos de elementos
#include "Cod_error.h"       // Definición de los códigos de error
#include "ajustes_rtu.h"     // Ajustes de modbus RTU
#include "comandos_tcp.h"    // Lista de comandos desde los HMI por modbus TCP
#include "comandos_rtu.h"    // Lista de comandos desde el WK0500 a los remotos por modbus RTU

//---------------------------------------------------------------------------------------------------------------------------

Rtc_Pcf8563 rtc;
Mudbus Mb;

// Puntero que apunta a la primera posición libre en la lista de elementos
unsigned int punteroEEprom = ((EEPROM.read (EP_PUNTERO_EEPROM)) <<8) | EEPROM.read (EP_PUNTERO_EEPROM+1);
unsigned int punteroRTU = EP_INICIO_ELEMENTOS; // Puntero para el sondeo de los remotos RTU
boolean modoNormal = true; // = false=> Modo sincronismo
unsigned int idTerminal; // identificador único del terminal que envía los comandos (1..15)
boolean comandoTCP= false; //true - se ha recibido un comando TCP

// objetos tipo
Elem_dig_t elemDig;
Elem_ana_t elemAna;
Termostato_t termostato;
Persiana_t persiana;
Phorario_t pHorario;
Phorario_ext_t pHorarioExt;
//-------------------------------------------------------------------------------------------------------

void setup()
{
  uint8_t mac[]     = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };
  uint8_t ip[]      = { 10, 0, 0, 166 };
  uint8_t gateway[] = { 192, 168, 1, 1 };
  uint8_t subnet[]  = { 255, 255, 0, 0 };
  
  // Inicializa el equipo si es una instalación nueva
  if (punteroEEprom >= EP_DIR_MAX_ELEM) volverValoresFabrica();
  
  
  Ethernet.begin(mac, ip, gateway, subnet);
  
  //Se inicializa modbus RTU
  remoto->register_array = regs;
  modbus_configure (BAUD, TIMEOUT, POLLING, RETRY_COUNT, TX_ENABLE_PIN, remotos, NUM_REMOTOS);
  Mb.R[MB_OFFSET_COMANDO] = 0xFFFF; // indica que no hay comando TCP
  
  pinMode (LED_AMARILLO,OUTPUT);
  pinMode (LED_ROJO,OUTPUT);
  
  
  for (byte i=0; i<=MAX_REMOTOS_RTU; i++) {
    MbRTU_Leido[i] = false;  //Ningún remoto ha sido sondeado todavía
    Mb.IR[i]= 0;             //Se borran los valores de estado de remotos RTU
  }
  
#ifdef debug 
  Serial.begin(9600); 
#endif  
}
//----------------------------------------------------------------------------------------

void loop()
{
  Mb.Run();     // Atiende el protocolo modbus TCP
  lecturaRTC(); // Lee el RTC (fecha y hora) y coloca los datos en los HR
  
  if (Mb.R[MB_OFFSET_COMANDO] != 0xFFFF) ejecutarComando(); // Si hay comando por TCP, lo ejecuta
  Mb.R[MB_OFFSET_COMANDO]=0xFFFF;                           // y lo borra
  
  resultadoRTU = modbus_update (remotos, cambio); // Sondea un remoto RTU
  cambio = resultadoRTU; // Al terminar un sondeo cambiamos al siguiente remoto
  if ((modoNormal && lecturaRTU || comandoTCP) && resultadoRTU) gestionRTU();
  //ojo!! cuando no sondea remotos se queda enviando el último comandoTCP
   
  
#ifdef debugAltas  
  mb_test_altas(); // Mapea la EEPROM en HR para depuración
#endif 


}
//-------------------------------------------------------------------------------------------------------
void gestionRTU(){
 
 #ifdef debugRTU
  // Los valores corresponden al último remoto sondeado
  Serial.print ("id: "); Serial.println (remoto->id);
  Serial.print ("Respuesta: "); Serial.println (resultadoRTU);
  Serial.print ("valor: "); Serial.println (regs[0]);
  Serial.print ("funcion: "); Serial.println (remoto->function);
  Serial.print ("HR: "); Serial.println (remoto->address);
  Serial.print ("Cant. HR: "); Serial.println (remoto->no_of_registers);
  Serial.print ("comando_1: "); Serial.println (comandoRTU[0],HEX);
  Serial.print ("comando_2: "); Serial.println (comandoRTU[1],HEX);
  Serial.println ("-----------------------------------");
#endif

byte dirMbAnterior= remoto->id;
unsigned int valorDevuelto = regs[0];
boolean commOK = resultadoRTU == 1;

MbRTU_Leido[dirMbAnterior]= true; // Para que no se vuelva a sondear en este ciclo

// Se carga el siguiente remoto a sondear
if (!comandoTCP && (EEPROM.read(EP_NUM_ELEMENTOS) >0)){
    lecturaCiclicaRTU(); 
    incrementarPuntero (&punteroRTU);
  } 
// o se carga el comando a enviar por TCP si es el caso
else for (int i=0; i < RTU_NO_HR; i++) regs[i]= comandoRTU[i];
    
 if (!comandoTCP && commOK) // sondeo anterior terminado OK
    Mb.IR[dirMbAnterior] = valorDevuelto; // Cargamos valor devuelto para Mb TCP
 
  senalizarRTU (commOK, dirMbAnterior);   
  comandoTCP= false;
}
//--------------------------------------------------------------------------------------------
void ejecutarComando (){

   idTerminal = (Mb.R[MB_OFFSET_COMANDO] & 0xF000) >> 12; // Se extrae el identificador de terminal (1..15)
   
#ifdef debugTCP  
  Serial.print ("Comando recibido: "); Serial.println (Mb.R[MB_OFFSET_COMANDO],HEX);
  Serial.print ("del id terminal: "); Serial.println (idTerminal);
  Serial.print ("al id elemento: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+1],HEX);
  Serial.print ("ARG1: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+2],HEX);
  Serial.print ("ARG2: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+3],HEX);
  Serial.print ("ARG3: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+4],HEX);
  Serial.print ("ARG4: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+5],HEX);
  Serial.print ("ARG5: "); Serial.println (Mb.R[MB_OFFSET_COMANDO+6],HEX);
  Serial.println ("-----------------------------------");
#endif  
  
 
  if (idTerminal == 0) return; // Terminal no identificado => No se ejecuta el comando
  
  switch (Mb.R[MB_OFFSET_COMANDO] & 0x0FFF) // Se extrae el comando recibido
  {    
    case oVOLVER_AJUSTES_FABRICA:  // Formato orden: Comando(HR1)        
     {
      volverValoresFabrica();
     }   
     break;
    
     case oAJUSTAR_FECHA: // Formato orden: Comando(HR1) + Año(HR2) + Mes(HR3) + dia(HR4) + dia de la semana(HR5)
        responder (NO_RESP);     
        ajustarFecha (Mb.R[MB_OFFSET_COMANDO+1],Mb.R[MB_OFFSET_COMANDO+2],
                      Mb.R[MB_OFFSET_COMANDO+3],Mb.R[MB_OFFSET_COMANDO+4]);   
     break;
          
     case oAJUSTAR_HORA: // Formato orden: Comando(HR1) + Hora(HR2) + Minuto(HR3) + Segundo(HR4)
        responder (NO_RESP);     
        ajustarHora (Mb.R[MB_OFFSET_COMANDO+1],Mb.R[MB_OFFSET_COMANDO+2],Mb.R[MB_OFFSET_COMANDO+3]);      
     break;    

     case oALTA_ELEMENTO: // Formato orden: Comando(HR1) + tipo(HR2) + idElemento(HR3) + escenas(HR4) + valoresEscenas(HR5)
     {
         responder (NO_RESP);
         byte tipo = Mb.R[MB_OFFSET_COMANDO+1];
         if (tipo==TIPO_DIGITAL || tipo==TIPO_LUZ_SIMPLE || tipo==TIPO_ENCHUFE) altaElementoDigital (Mb.R);
         if (tipo==TIPO_ANALOGICO) altaElementoAnalogico (Mb.R);
         if (tipo==TIPO_PROG_HORARIO) altaProgramaHorario (Mb.R);
         if (tipo==TIPO_TERMOSTATO) altaTermostato (Mb.R);
         if (tipo==TIPO_PERSIANA) altaPersiana (Mb.R);
     }   
     break;   
      
      case oACTIVAR_ENCHUFE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_ENCHUFE, RTU_ACTIVACION_REMOTO);
      break;
     
      case oDESACTIVAR_ENCHUFE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_ENCHUFE, RTU_DESACTIVACION_REMOTO);
      break; 
      
      case oACTIVAR_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_ACTIVACION_PERSIANA);
      break;
     
      case oDESACTIVAR_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_DESACTIVACION_PERSIANA);
      break; 
      
      case oDETENER_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_DETENER_PERSIANA);
      break; 
      
      case oACTIVAR_LUZ_SIMPLE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_SIMPLE, RTU_ACTIVACION_REMOTO);
      break;  
      
      case oDESACTIVAR_LUZ_SIMPLE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);   
         comandoTCP = true;
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_SIMPLE, RTU_DESACTIVACION_REMOTO); 
      break;  
      
      case oACTIVAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         comandoTCP = true;
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_ACTIVACION_REMOTO,0);
      break;  
      
      case oDESACTIVAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);  
         comandoTCP = true; 
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_DESACTIVACION_REMOTO,0); 
      break; 
      
      case oREGULAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2) + valorRegulación (HR3)
         responder (NO_RESP);
         comandoTCP = true;   
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_REGULACION_REMOTO, Mb.R[MB_OFFSET_COMANDO+2]); 
      break;
      
      case oEJECUTAR_ESCENA:  // Formato orden: Comando (HR1) + Número de escena(HR2)
         responder (NO_RESP); 
         comandoTCP = true; 
         EjecutarEscena (Mb.R[MB_OFFSET_COMANDO+1]); 
      break;  
       
      case oAJUSTAR_CONSIGNA_DIA: // Formato orden: Comando (HR1) + idElemento(HR2) + consignaDia(HR3) (grados)
         responder (NO_RESP);
         comandoTCP = true;
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_CONSIGNA_DIA);
      break;
      
      case oAJUSTAR_MAX_NOCHE: // Formato orden: Comando (HR1) + idElemento(HR2) + Temp. máx en modo noche(HR3) (grados)
         responder (NO_RESP);
         comandoTCP = true;
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_MAX_NOCHE);
      break;
      
      case oAJUSTAR_MIN_NOCHE: // Formato orden: Comando(HR1) + idElemento(HR2) + Temp. min en modo noche(HR3) (grados)
         responder (NO_RESP);
         comandoTCP = true;
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_MIN_NOCHE);
      break;
      
      case oAJUSTAR_VELOCIDAD_VENT: // Formato orden: Comando(HR1) + idElemento(HR2) + Velocidad ventilador(HR3)
         responder (NO_RESP);
         comandoTCP = true;
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_VELOCIDAD_VENT);
      break;
      
      case oMODO_SINCRONIZACION: // Formato orden: Comando(HR1) + Tipo(HR2)
         responder (NO_RESP);
         modoNormal = false;
         ListaDeTipo(Mb.R[MB_OFFSET_COMANDO+1]);      
      break;
      
      case oMODO_NORMAL: // Formato orden: Comando(HR1)
         responder (NO_RESP);
         for (int i=0;i<=MAX_REMOTOS_RTU;i++) Mb.IR[i]=0; //Borrado de la zona de input registers
         for (byte i=0; i<=MAX_REMOTOS_RTU; i++) MbRTU_Leido[i] = false; //Ningún remoto sondeado
         punteroRTU = EP_INICIO_ELEMENTOS; //Inicializamos puntero de lectura RTU
         modoNormal = true;
      break;
  }
}

//---------------------------------------------------------------------------------------------------
// Envía las órdenes a los elementos de tipo termostato por modbus RTU
// A continuación realiza una lectura del mismo termostato al que envió el comando
// Parámetros:
//   - id_elemento: Identificador único del termostato. Coincide con su dirección MB desplazada 4 bits
//   - orden: Código del comando recibido por Modbus TCP
//----------------------------------------------------------------------------------------------------
void actuarTermostato (int id_elemento, int orden){

  if (tipoElemento (busquedaElemento (id_elemento)) == TIPO_TERMOSTATO){ //Verifica que el tipo coincide
       
      // Construimos parte del comando RTU   
      remoto->id = id_elemento >> 4; // dirección modbus RTU del termostato
      remoto->function = PRESET_SINGLE_REGISTER;
      remoto->no_of_registers = 1; 
      comandoRTU[0]= Mb.R[MB_OFFSET_COMANDO+2]; // Valor a enviar 

      
      switch (orden) //Se selecciona la dirección modbus RTU del termostato según el comando
      {
        case oAJUSTAR_CONSIGNA_DIA:   
         remoto->address = RTU_CONSIGNA_DIA;
        break;
        
        case oAJUSTAR_MAX_NOCHE:  
         remoto->address = RTU_MAX_NOCHE;
        break;
        
        case oAJUSTAR_MIN_NOCHE:  
         remoto->address = RTU_MIN_NOCHE;
        break;
        
        case oAJUSTAR_VELOCIDAD_VENT:  
         remoto->address = RTU_VELOCIDAD_VENT;
        break;        
      }  
      
#ifdef debugComandosRTU     
      Serial.println ("Configurada orden a termostato: ");
      Serial.print ("dirModbus: "); Serial.println (remoto->id);
      Serial.print ("Función: "); Serial.println (remoto->function);
      Serial.print ("dirHR_Remoto: "); Serial.println (remoto->address);
      Serial.print ("Num_HR: "); Serial.println (remoto->no_of_registers);
      Serial.print ("Orden a enviar: "); Serial.println (comandoRTU[0]);
      Serial.print ("Valor a enviar: "); Serial.println (comandoRTU[1]);
      Serial.println ("------------------------------------------");
#endif  

  }
  else responder (ERR_Tipo); // Tipo del idElemento no coincide con un TIPO_TERMOSTATO
}
//-------------------------------------------------------------------------------------------------------
// Envía las órdenes a los elementos de tipo digital (luz,enchufe...) por modbus RTU
// A continuación realiza una lectura del mismo elemento al que envió el comando
// Parámetros:
//   - id_elemento: Identificador único del elemento. Fomado por DirMB(12 bit) + canal (4 bit)
//   - tipo: Identificador de tipo del elemento
//   - orden: Código del comando recibido por Modbus TCP
//----------------------------------------------------------------------------------------------------
void actuarElementoDigital (int id_elemento, int tipo, int orden){

  if (tipoElemento (busquedaElemento (id_elemento)) == tipo){
      comandoRTU[0]= orden;   
      comandoRTU[1]= 1 << (id_elemento & 0x000F); 
      remoto->id = id_elemento >> 4;
      remoto->function = PRESET_MULTIPLE_REGISTERS;
      remoto->address = RTU_HR_COMANDO;
      remoto->no_of_registers = RTU_NO_HR;

#ifdef debugComandos     
      Serial.println ("Configurada orden a elemento digital: ");
      Serial.print ("idElemento: "); Serial.println (id_elemento,HEX);
      Serial.print ("dirModbus: "); Serial.println (remoto->id);
      Serial.print ("Función: "); Serial.println (remoto->function);
      Serial.print ("dirHR_Remoto: "); Serial.println (remoto->address);
      Serial.print ("Num_HR: "); Serial.println (remoto->no_of_registers);
      Serial.print ("ComandoAlRemoto: "); Serial.println (comandoRTU[0],HEX);
      Serial.print ("Valor a enviar: "); Serial.println (comandoRTU[1],HEX);
      Serial.println ("------------------------------------------");
#endif      
  }
  else responder (ERR_Tipo);    
}

//-------------------------------------------------------------------------------------------------------
// Envía las órdenes a los elementos de tipo persiana por modbus RTU
// A continuación realiza una lectura del mismo elemento al que envió el comando
// Parámetros:
//   - id_elemento: Identificador único del elemento. Fomado por DirMB(12 bit) + canal (4 bit)
//   - tipo: Identificador de tipo del elemento
//   - orden: Código del comando recibido por Modbus TCP
//----------------------------------------------------------------------------------------------------
void actuarPersiana (int id_elemento, int tipo, int orden){

  if (tipoElemento (busquedaElemento (id_elemento)) == tipo){
      comandoRTU[0]= orden;    
      remoto->id = id_elemento >> 4;
      remoto->function = PRESET_MULTIPLE_REGISTERS;
      remoto->address = RTU_HR_COMANDO;
      remoto->no_of_registers = 1;

#ifdef debugComandos    
      Serial.println ("Configurada orden a persiana: ");
      Serial.print ("idElemento: "); Serial.println (id_elemento,HEX);
      Serial.print ("dirModbus: "); Serial.println (remoto->id);
      Serial.print ("Función: "); Serial.println (remoto->function);
      Serial.print ("dirHR_Remoto: "); Serial.println (remoto->address);
      Serial.print ("Num_HR: "); Serial.println (remoto->no_of_registers);
      Serial.print ("Comando a enviar: "); Serial.println (comandoRTU[0],HEX);
      Serial.println ("------------------------------------------");
#endif  

  }
  else responder (ERR_Tipo);    
}
//-------------------------------------------------------------------------------------------------------
// Envía las órdenes a los elementos de tipo analógico por modbus RTU
// A continuación realiza una lectura del mismo elemento al que envió el comando
// Parámetros:
//   - id_elemento: Identificador único del elemento. Fomado por DirMB(12 bit) + canal (4 bit)
//   - tipo: Identificador de tipo del elemento
//   - orden: Código del comando recibido por Modbus TCP
//   - valor: Valor de regulación del elemento 0 = 0%  ,  255  = 100%
//----------------------------------------------------------------------------------------------------
void actuarElementoAnalogico (int id_elemento, int tipo, int orden, unsigned int valor){
   
  if (valor > 255) {
      responder (ERR_valor_fuera_de_rango);
      return;
  }
  if (tipoElemento (busquedaElemento (id_elemento)) == tipo){
      comandoRTU[0]= orden;   
      comandoRTU[1]= 1 << (id_elemento & 0x000F); 
      comandoRTU[2]= valor;
      remoto->id = id_elemento >> 4;
      remoto->function = PRESET_MULTIPLE_REGISTERS;
      remoto->address = RTU_HR_COMANDO;
      remoto->no_of_registers = RTU_NO_HR;
      
      if (orden == RTU_REGULACION_REMOTO) remoto->no_of_registers = 3;


#ifdef debugComandos     
      Serial.println ("Configurada orden a elemento analógico: ");
      Serial.print ("idElemento: "); Serial.println (id_elemento,HEX);
      Serial.print ("dirModbus: "); Serial.println (remoto->id);
      Serial.print ("Función: "); Serial.println (remoto->function);
      Serial.print ("dirHR_Remoto: "); Serial.println (remoto->address);
      Serial.print ("Num_HR: "); Serial.println (remoto->no_of_registers);
      Serial.print ("ComandoAlRemoto: "); Serial.println (comandoRTU[0],HEX);
      Serial.print ("Canal: "); Serial.println (comandoRTU[1]);
      Serial.print ("Valor a enviar: "); Serial.println (comandoRTU[2]);
      Serial.println ("------------------------------------------");
#endif  

  }
  else responder (ERR_Tipo);    
}
//-----------------------------------------------------------------------------------------------
//Busca un elemento en la EEPROM a partir del idElemento
//Devuelve un puntero a la posición del elemento
//o -1 si el elemento no se encuentra en la memoria
//-----------------------------------------------------------------------------------------------

int busquedaElemento(int id){
  
  byte id_alto= id >> 8;
  byte id_bajo= id & 0x00FF;
  unsigned int  punteroBusqueda = EP_INICIO_ELEMENTOS;
  byte numElementos = EEPROM.read (EP_NUM_ELEMENTOS);
  byte tipo,id_alto_leido,id_bajo_leido;
  
  for (byte i=1; i<=numElementos; i++) {
    tipo=EEPROM.read (punteroBusqueda);
    id_bajo_leido=EEPROM.read (punteroBusqueda+1);
    id_alto_leido=EEPROM.read (punteroBusqueda+2);
    if (id_alto==id_alto_leido && id_bajo==id_bajo_leido) return punteroBusqueda; //id encontrado
    else incrementarPuntero (&punteroBusqueda);  
  }  
 return -1; // id no encontrado 
}
//--------------------------------------------------------------------------------------------
// Devuelve el tipo de un elemento a partir de la situación del elemento en la EEPROM
//--------------------------------------------------------------------------------------------
byte tipoElemento (int p){
  
  if (p>0 && p<DIR_MAX_EEPROM) return EEPROM.read (p);
  else return 0; // No existe ningún tipo con id 0
} 
//--------------------------------------------------------------------------------------------
// Lee el reloj en tiempo real y coloca los valores en los holding registers a partir
// del registro MB_OFFSET_FECHA_HORA
//--------------------------------------------------------------------------------------------
void lecturaRTC() {
  rtc.formatTime();
  rtc.formatDate();
  Mb.R[MB_OFFSET_FECHA_HORA] = rtc.getYear();
  Mb.R[MB_OFFSET_FECHA_HORA+1] = rtc.getMonth();
  Mb.R[MB_OFFSET_FECHA_HORA+2] = rtc.getDay();
  Mb.R[MB_OFFSET_FECHA_HORA+3] = rtc.getHour();
  Mb.R[MB_OFFSET_FECHA_HORA+4] = rtc.getMinute();
  Mb.R[MB_OFFSET_FECHA_HORA+5] = rtc.getSecond();
}
//-----------------------------------------------------------------------------------------
// Ajusta la fecha del RTC
// dia_semana: 1 = Domingo
// siglo: 0 => Siglo XXI
//--------------------------------------------------------------------------------------------
void ajustarFecha (int ano,int mes,int dia,int dia_semana) {
  if (dia>=1 && dia<=31 && mes>=1 && mes <=12 && ano>0 && dia_semana >=1 && dia_semana <=7)
  {
    rtc.setDate(dia, dia_semana, mes, 0, ano); //dia, dia_semana, mes, siglo, año
    responder (OK);
  }
  else responder (ERR_Fecha);
}
//-----------------------------------------------------------------------------------------
// Ajusta la hora del RTC
//--------------------------------------------------------------------------------------------
void ajustarHora (int hora,int minuto,int segundo) {
  if (hora >=0 && hora <=23 && minuto >=0 && minuto <=59 && segundo >=0 && segundo <=59)
  {
    rtc.setTime(hora,minuto,segundo); responder (OK);
  }
  else responder (ERR_Hora);
}
//--------------------------------------------------------------------------------------------
void EjecutarEscena (byte escena){ // No sirve ahora. Rehacer completo
  
  if (escena > 16 || escena < 1){
    responder (ERR_Escena_no_valida);
    return;
  }
  
  int e = 1 << (escena-1);  
  byte numElementos = EEPROM.read (EP_NUM_ELEMENTOS);
  unsigned int punteroBusqueda = EP_INICIO_ELEMENTOS;
  byte tipo;
  
  for (int i=1; i<=numElementos; i++) {
    tipo=tipoElemento (punteroBusqueda);
    if (tipo== TIPO_DIGITAL || tipo== TIPO_LUZ_SIMPLE || tipo== TIPO_ENCHUFE) {
      eeprom_read_block((void*)&elemDig, (void*)punteroBusqueda, sizeof(elemDig));
      if (elemDig.escenas & elemDig.valoresEscenas & e) actuarElementoDigital (elemDig.idElemento, tipo, RTU_ACTIVACION_REMOTO); 
      if ((elemDig.escenas & e) && !(elemDig.valoresEscenas & e)) actuarElementoDigital (elemDig.idElemento, tipo, RTU_DESACTIVACION_REMOTO);
    }
    if (tipo== TIPO_ANALOGICO) {
      eeprom_read_block((void*)&elemAna, (void*)punteroBusqueda, sizeof(elemAna));
      if (elemAna.escenas & e) actuarElementoAnalogico (elemAna.idElemento, tipo, RTU_REGULACION_REMOTO, elemAna.valoresEscenas[escena-1]); 
    }
    
      incrementarPuntero (&punteroBusqueda);
      
    }
}
//--------------------------------------------------------------------------------------------
// Recorre la memoria EEPROM y busca todos los identificadores de elemento del tipo solicitado
// Coloca los identificadores en los input registers (a partir de la dirección 1)
// En el input register 0 coloca el identificador de tipo solicitado
// El resto de input registers se ponen a 0
//--------------------------------------------------------------------------------------------

void ListaDeTipo(byte tipoSolicitado){
  
  
  unsigned int  punteroBusqueda = EP_INICIO_ELEMENTOS; 
  byte numElementos = EEPROM.read (EP_NUM_ELEMENTOS);
  byte tipo;
  byte j=0;
  Mb.IR[0]=tipoSolicitado;
  
  for (byte i=1; i<=numElementos; i++) {   
    tipo=EEPROM.read (punteroBusqueda);
    if (tipoSolicitado == tipo) {
      j++;
      Mb.IR[j] = EEPROM.read (punteroBusqueda+1) | (EEPROM.read (punteroBusqueda+2)) << 8;     
    }
    incrementarPuntero (&punteroBusqueda);
    
  }
j++;
for (byte i=j; i<=124; i++) Mb.IR[i]=0; // Se borran el resto de los input registers
}
//--------------------------------------------------------------------------------------------
// Lee un holding register de un remoto RTU. Devuelve el valor leído.
// dirMb: Dirección modbus RTU del remoto
// tipo: tipo de remoto (I/0, termostato, persiana, etc.)
// La dirección del registro leído depende del tipo de remoto
// En el caso de un termostato se realizan 3 lecturas para obtener:
//     - Temperatura actual (décima de grado)
//     - Temperatura de consigna (grados)
//     - Modo día/noche
// En el caso más general, la información dependerá del remoto
//--------------------------------------------------------------------------------------------

unsigned int leerRegistroRemoto (byte dirMb, byte tipo){

  remoto->id = dirMb;
  remoto->function = READ_HOLDING_REGISTERS;
  remoto->no_of_registers = 1;
  

  switch (tipo){
    case TIPO_DIGITAL:
    case TIPO_LUZ_SIMPLE:
    case TIPO_ENCHUFE:
      remoto->address = RTU_HR_ADDR; 
      break;
    case TIPO_TERMOSTATO:
      remoto->address = RTU_TEMPERATURA;
      break;
  } 
  contadorRTU++; 
} 
//--------------------------------------------------------------------------------------------------
void lecturaCiclicaRTU (){
    
  byte numElementos = EEPROM.read (EP_NUM_ELEMENTOS);
  byte tipo; 
  unsigned int dirMB; 

#ifdef debugRTU  
  Serial.println ("Lectura Ciclica RTU");
  Serial.print ("PunteroRTU: "); Serial.println (punteroRTU);
  Serial.print ("PunteroEEprom: "); Serial.println (punteroEEprom);
  Serial.print ("ContadorRTU: "); Serial.println (contadorRTU);
  Serial.println ("----------------------------------");
#endif  

   // Al llegar al último elemento, volvemos a comenzar
   if (punteroRTU >= punteroEEprom){
      punteroRTU = EP_INICIO_ELEMENTOS;  
      for (byte i=0; i<=numElementos; i++) MbRTU_Leido[i] = false;
      contadorRTU = 0;
   }
  tipo=EEPROM.read (punteroRTU);
  dirMB = (byte)((word (EEPROM.read (punteroRTU+2), EEPROM.read (punteroRTU+1))) >> 4);
  // Buscamos el próximo elemento RTU a sondear
  while (((tipo == TIPO_PROG_HORARIO) || MbRTU_Leido [dirMB] ) && (punteroRTU < punteroEEprom) ) {
    incrementarPuntero (&punteroRTU);
    tipo=EEPROM.read (punteroRTU);
    dirMB = (byte)((word (EEPROM.read (punteroRTU+2), EEPROM.read (punteroRTU+1))) >> 4);
  }
   if (punteroRTU < punteroEEprom) leerRegistroRemoto (dirMB,tipo);     
      
   
}
//--------------------------------------------------------------------------------------------
// Realiza el alta (inserción en EEPROM) de un nuevo elemento digital
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaElementoDigital (int* p){
  
  Elem_dig_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idElemento = *(p++);
  nuevoElemento.escenas = *(p++);
  nuevoElemento.valoresEscenas = *(p++);
  
  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if (tipoElemento (busquedaElemento (nuevoElemento.idElemento & 0xFFF0) == TIPO_TERMOSTATO )) responder (ERR_IdDuplicado);
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU) responder (ERR_DirMbNoValida);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
#ifdef debugAltas   
   Serial.println ("Alta elemento digital: ");
   Serial.print ("idElemento: "); Serial.println (nuevoElemento.idElemento, HEX);
   Serial.print ("Puntero: "); Serial.println (punteroEEprom, HEX);
   Serial.println ("-----------------------------------");  
#endif    
  }
}
//--------------------------------------------------------------------------------------------
// Realiza el alta (inserción en EEPROM) de un nuevo elemento analógico
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaElementoAnalogico (int* p){
  
  Elem_ana_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idElemento = *(p++);
  nuevoElemento.escenas = *(p++);
  for (int i=0; i<=15; i++)
  nuevoElemento.valoresEscenas[i] = *(p++);

  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if (tipoElemento (busquedaElemento (nuevoElemento.idElemento & 0xFFF0) == TIPO_TERMOSTATO )) responder (ERR_IdDuplicado);
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU) responder (ERR_DirMbNoValida);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
#ifdef debugAltas   
   Serial.println ("Alta elemento analogico: ");
   Serial.print ("idElemento: "); Serial.println (nuevoElemento.idElemento, HEX);
   Serial.print ("Puntero: "); Serial.println (punteroEEprom, HEX);
   Serial.println ("-----------------------------------");  
#endif    
  }
}
//--------------------------------------------------------------------------------------------
// Realiza el alta (inserción en EEPROM) de un nuevo programa horario
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaProgramaHorario (int* p){
  
  Phorario_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idPrograma = *(p++); 
  nuevoElemento.idElemento = *(p++);
  nuevoElemento.activacion = *(p++);
  nuevoElemento.hora = *(p++);
  nuevoElemento.dia_semana = *(p++);
  nuevoElemento.valor = *(p++);
 
  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
#ifdef debugAltas   
   Serial.println ("Alta programa horario: ");
   Serial.print ("idElemento: "); Serial.println (nuevoElemento.idElemento, HEX);
   Serial.print ("Puntero: "); Serial.println (punteroEEprom, HEX);
   Serial.println ("-----------------------------------");  
#endif   
  }
}

//--------------------------------------------------------------------------------------------
// Realiza el alta (inserción en EEPROM) de una nueva persiana
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaPersiana (int* p){
  
  Persiana_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idElemento = *(p++);
  
  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if (tipoElemento (busquedaElemento (nuevoElemento.idElemento & 0xFFF0) == TIPO_TERMOSTATO )) responder (ERR_IdDuplicado);
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU) responder (ERR_DirMbNoValida);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
#ifdef debugAltas  
   Serial.println ("Alta persiana: ");
   Serial.print ("idElemento: "); Serial.println (nuevoElemento.idElemento, HEX);
   Serial.print ("Puntero: "); Serial.println (punteroEEprom, HEX);
   Serial.println ("-----------------------------------");
#endif     
  }
}

//--------------------------------------------------------------------------------------------
// Realiza el alta (inserción en EEPROM) de un nuevo termostato
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaTermostato (int* p){
  
  Termostato_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idElemento = *(p++);
  
  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if ((nuevoElemento.idElemento & 0x000F) != 0) responder (ERR_DirMbNoValida);
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU) responder (ERR_DirMbNoValida);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
   
#ifdef debugAltas  
   Serial.println ("Alta termostato: ");
   Serial.print ("idElemento: "); Serial.println (nuevoElemento.idElemento, HEX);
   Serial.print ("Puntero: "); Serial.println (punteroEEprom, HEX);
   Serial.println ("-----------------------------------");
#endif   
  }
}
//--------------------------------------------------------------------------------------------
// Actualiza el puntero de altas en RAM y EEPROM
// Actualiza el número de elementos de alta en EEPROM (+1)
// tamano: Número de bytes a incrementar el puntero (tamaño en bytes del último elemento insertado)
//--------------------------------------------------------------------------------------------

void actualizarPuntero (int tamano){
 punteroEEprom+=tamano;
 EEPROM.write (EP_PUNTERO_EEPROM, punteroEEprom >> 8);
 EEPROM.write (EP_PUNTERO_EEPROM+1, punteroEEprom & 0x00FF);
 EEPROM.write (EP_NUM_ELEMENTOS, EEPROM.read (EP_NUM_ELEMENTOS)+1);
}

//--------------------------------------------------------------------------------------------

void mb_test_altas (){
  Mb.R [MB_TEST_ALTAS-2] = EEPROM.read (EP_NUM_ELEMENTOS);
  Mb.R [MB_TEST_ALTAS-1] = punteroEEprom;
  for (int i=MB_TEST_ALTAS; i<=MB_TEST_ALTAS+100; i++) 
   Mb.R [i] = EEPROM.read (EP_INICIO_ELEMENTOS+i-MB_TEST_ALTAS);
}
//--------------------------------------------------------------------------------------------

void responder (int codigo){

   Mb.R[MB_OFFSET_RESPUESTA +idTerminal] = Mb.R[MB_OFFSET_COMANDO] | (codigo << 8);
#ifdef debugRespuestas
   Serial.print ("Respuesta: "); Serial.print (codigo); Serial.print (" al comando: "); 
   Serial.println (Mb.R[MB_OFFSET_COMANDO], HEX);
   Serial.println ("------------------------------");
#endif   
}   
//---------------------------------------------------------------------------------------------   

void senalizarRTU (boolean estado, byte direccion){

 digitalWrite (LED_ROJO,estado);

 if (estado) bitSet (Mb.IR[IR_OFFSET_COMOK+direccion/16], direccion % 16);  
 else bitClear (Mb.IR[IR_OFFSET_COMOK+direccion/16], direccion % 16);    
}

//--------------------------------------------------------------------------------------------- 

void incrementarPuntero (unsigned int *p){
 
byte tipo= EEPROM.read (*p);

switch (tipo){
      
      case TIPO_DIGITAL:
      case TIPO_LUZ_SIMPLE:
      case TIPO_ENCHUFE:
        *p+=sizeof(elemDig);
      break;
      
      case TIPO_ANALOGICO:
        *p+=sizeof(elemAna);
      break;
      
      case TIPO_PROG_HORARIO:
        *p+=sizeof(pHorario);
      break;
      
      case TIPO_PERSIANA:
        *p+=sizeof(persiana);
        
      case TIPO_TERMOSTATO:
        *p+=sizeof(termostato);     
      }
}
//---------------------------------------------------------------------------------------------
void volverValoresFabrica(){
      responder (NO_RESP); 
      //Borrado completo de la memoria EEPROM
      for (unsigned int i=0; i<=DIR_MAX_EEPROM; i++) EEPROM.write (i,0xFF); 
      //Se coloca puntero al inicio de la zona de elementos
      EEPROM.write (EP_PUNTERO_EEPROM+1,EP_INICIO_ELEMENTOS & 0x00FF); 
      EEPROM.write (EP_PUNTERO_EEPROM,EP_INICIO_ELEMENTOS >> 8);
      // Se carga el puntero en RAM 
      punteroEEprom = (EEPROM.read (EP_PUNTERO_EEPROM))<<8 | EEPROM.read (EP_PUNTERO_EEPROM+1);
      EEPROM.write (EP_NUM_ELEMENTOS,0); // Cantidad de elementos = 0
      punteroRTU = EP_INICIO_ELEMENTOS;
      responder (OK);
      // faltan ajustes por defecto (IP, máscara, parámetros RTU...) y reiniciar
}
//---------------------------------------------------------------------------------------------
