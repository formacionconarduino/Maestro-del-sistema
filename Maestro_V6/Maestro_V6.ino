//---------------------------------------------------------------------------------------------------------------------------
//           Maestro domótica V 0.1.0.1
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
#define debugAltas        // Depuración para altas de nuevos elementos
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
#include "comAut.h"          // Comandos automáticos (procedentes de las entradas del sistema domótico)

//---------------------------------------------------------------------------------------------------------------------------

Rtc_Pcf8563 rtc;
Mudbus Mb;

// Puntero que apunta a la primera posición libre en la lista de elementos
unsigned int punteroEEprom = ((EEPROM.read (EP_PUNTERO_EEPROM)) <<8) | EEPROM.read (EP_PUNTERO_EEPROM+1);
unsigned int punteroRTU = EP_INICIO_ELEMENTOS; // Puntero para el sondeo de los remotos RTU
boolean modoNormal = true; // = false=> Modo sincronismo
unsigned int idTerminal; // identificador único del terminal que envía los comandos (1..15)


unsigned int tarjetaRFID[2]; // Guarda la tarjeta RFID leída en el terminal de monedero
unsigned int leyendoTipo;
unsigned int dirMB_Monedero;

// objetos tipo
Elem_dig_t elemDig;
Elem_ana_t elemAna;
Termostato_t termostato;
Persiana_t persiana;
Phorario_t pHorario;
Phorario_ext_t pHorarioExt;
Monedero_t monedero;
//-------------------------------------------------------------------------------------------------------

void setup()
{
  uint8_t mac[]     = { 0x90, 0xA2, 0xDA, 0x00, 0x51, 0x06 };
  uint8_t ip[]      = { 10, 0, 0, 166 };
  uint8_t gateway[] = { 10, 0, 0, 224 };
  uint8_t subnet[]  = { 255, 255, 255, 0 };
  
  // Inicializa el equipo si es una instalación nueva
  if (punteroEEprom >= EP_DIR_MAX_ELEM) volverValoresFabrica ();
  
  
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
  if (tarjetaRFID[0] != 0 || tarjetaRFID[1] != 0) procesarMonedero();
  
  if (Mb.R[MB_OFFSET_COMANDO] != 0xFFFF) ejecutarComando(); // Si hay comando por TCP, lo ejecuta
  Mb.R[MB_OFFSET_COMANDO]=0xFFFF;                           // y lo borra
  
  resultadoRTU = modbus_update (remotos, cambio); // Sondea un remoto RTU
  cambio = resultadoRTU; // Al terminar un sondeo cambiamos al siguiente remoto
  if ((modoNormal && lecturaRTU || (tamanoColaRTU()>0)) && resultadoRTU) gestionRTU();
  
  //ojo!! cuando no sondea remotos se queda enviando el último comandoTCP
   
  
#ifdef debugAltas  
  mb_test_altas(); // Mapea la EEPROM en HR para depuración
#endif 


}
//----------------------------------------------------------------------------
void gestionRTU(){
 
 #ifdef debugRTU
  // Los valores corresponden al último remoto sondeado
  Serial.print ("id: "); Serial.println (remoto->id);
  Serial.print ("Respuesta: "); Serial.println (resultadoRTU);
  Serial.print ("Regs[0]: "); Serial.println (regs[0]);
  Serial.print ("Regs[1]: "); Serial.println (regs[1]);
  Serial.print ("funcion: "); Serial.println (remoto->function);
  Serial.print ("HR: "); Serial.println (remoto->address);
  Serial.print ("Cant. HR: "); Serial.println (remoto->no_of_registers);
  Serial.println ("-----------------------------------");
#endif

byte idAnterior = remoto->id;
byte funcionAnterior = remoto->function;
byte numRegistrosAnterior = remoto->no_of_registers;
boolean lecturaOK = (resultadoRTU == 1) && (funcionAnterior == 3);
boolean escrituraOK = (resultadoRTU == 1) && ((funcionAnterior == 16) || (funcionAnterior == 6));
unsigned int tipo=leyendoTipo;
boolean respuestaAutomatica = (tipo==TIPO_MONEDERO) && lecturaOK && (regs[0] !=0 || regs[1] !=0);

 senalizarRTU (lecturaOK || escrituraOK, idAnterior);  

 if ((funcionAnterior == 3) && (tipo != TIPO_TERMOSTATO)) MbRTU_Leido[idAnterior]= true; // Para que no se vuelva a sondear en este ciclo
 
 if (lecturaOK){
   
   switch (tipo){
     
     case TIPO_DIGITAL:
     case TIPO_ANALOGICO:
     case TIPO_ENCHUFE:
     case TIPO_LUZ_SIMPLE:
     case TIPO_LUZ_REGULADA:
     case TIPO_PERSIANA:
     // comprobar aquí las respuestas automáticas
     //comprobarRespuestaAutomatica (idAnterior, Mb.IR, regs);
       for (byte k=0; k < CANT_IR_ESTADO; k++) Mb.IR[idAnterior * CANT_IR_ESTADO +k] = regs[k]; // Cargamos valor devuelto para Mb TCP
     break;
     case TIPO_TERMOSTATO:
     {
       if ((funcionAnterior == 3) && (remoto->address == RTU_TEMPERATURA)){
         anadirOrdenColaRTU (READ_HOLDING_REGISTERS, idAnterior, RTU_CONSIGNA_DIA, 1, comandoRTU);
         anadirOrdenColaRTU (READ_HOLDING_REGISTERS, idAnterior, RTU_MODO_DIA, 1, comandoRTU);
         Mb.IR[idAnterior * CANT_IR_ESTADO] = (Mb.IR[idAnterior * CANT_IR_ESTADO] & 0xFE00) | (regs[0] & 0x01FF);
       }
       else if ((funcionAnterior == 3) && (remoto->address == RTU_CONSIGNA_DIA))
         Mb.IR[idAnterior * CANT_IR_ESTADO] = (Mb.IR[idAnterior * CANT_IR_ESTADO] & 0x81FF) | (regs[0] & 0x7E00);
       else if ((funcionAnterior == 3) && (remoto->address == RTU_MODO_DIA))
         if (bitRead (regs[0],0)) Mb.IR[idAnterior * CANT_IR_ESTADO] = Mb.IR[idAnterior * CANT_IR_ESTADO] | 0x8000;  
            else Mb.IR[idAnterior * CANT_IR_ESTADO] = Mb.IR[idAnterior * CANT_IR_ESTADO] & 0x7FFF;           
     }     
     break;
     
     case TIPO_MONEDERO:
       tarjetaRFID[0]= regs[0];
       tarjetaRFID[1]= regs[1];
       Mb.R[16]= regs[0]; //pendientes de definir
       Mb.R[17]= regs[1];
       dirMB_Monedero=idAnterior;
     break;
   }
 }

if ((tamanoColaRTU()==0) && !respuestaAutomatica && (EEPROM.read(EP_NUM_ELEMENTOS) >0)){
    lecturaCiclicaRTU(); 
    incrementarPuntero (&punteroRTU);
  } 
  
if (tamanoColaRTU () > 0) {
    remoto->function = colaRTU[0][0];
    remoto->id = colaRTU[0][1];
    remoto->address = colaRTU[0][2];
    remoto->no_of_registers = colaRTU[0][3];
    for (int i=0; i < remoto->no_of_registers; i++) regs[i]= colaRTU[0][4+i];
    eliminarOrdenColaRTU();
}
  
}

//------------------------------------------------------------------------------------------------------------------

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
      volverValoresFabrica ();
     }   
     break;
     
     case oCOMANDO_DIRECTO_RTU: // Formato orden: Comando (HR1) + Dir_MB_RTU (HR2) + DirHR (HR3) + NUM_HR (HR4)+ ComandoRTU[1] (HR5) + ...
       responder (NO_RESP);
       for (byte i=0; i < Mb.R[MB_OFFSET_COMANDO+3]; i++) comandoRTU [i] = Mb.R[MB_OFFSET_COMANDO+4+i];
       anadirOrdenColaRTU (PRESET_MULTIPLE_REGISTERS, Mb.R[MB_OFFSET_COMANDO+1], Mb.R[MB_OFFSET_COMANDO+2], Mb.R[MB_OFFSET_COMANDO+3], comandoRTU);
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
         if (tipo==TIPO_MONEDERO) altaMonedero (Mb.R);
     }   
     break;   
      
      case oACTIVAR_ENCHUFE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_ENCHUFE, RTU_ACTIVACION_REMOTO);
      break;
     
      case oDESACTIVAR_ENCHUFE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_ENCHUFE, RTU_DESACTIVACION_REMOTO);
      break; 
      
      case oACTIVAR_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_ACTIVACION_PERSIANA);
      break;
     
      case oDESACTIVAR_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_DESACTIVACION_PERSIANA);
      break; 
      
      case oDETENER_PERSIANA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarPersiana (Mb.R[MB_OFFSET_COMANDO+1], TIPO_PERSIANA, RTU_DETENER_PERSIANA);
      break; 
      
      case oACTIVAR_LUZ_SIMPLE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_SIMPLE, RTU_ACTIVACION_REMOTO);
      break;  
      
      case oDESACTIVAR_LUZ_SIMPLE: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);   
         actuarElementoDigital (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_SIMPLE, RTU_DESACTIVACION_REMOTO); 
      break;  
      
      case oACTIVAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_ACTIVACION_REMOTO,0);
      break;  
      
      case oDESACTIVAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2)
         responder (NO_RESP);  
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_DESACTIVACION_REMOTO,0); 
      break; 
      
      case oREGULAR_LUZ_REGULADA: // Formato orden: Comando (HR1) + idElemento(HR2) + valorRegulación (HR3)
         responder (NO_RESP); 
         actuarElementoAnalogico (Mb.R[MB_OFFSET_COMANDO+1], TIPO_LUZ_REGULADA, RTU_REGULACION_REMOTO, Mb.R[MB_OFFSET_COMANDO+2]); 
      break;
      
      case oEJECUTAR_ESCENA:  // Formato orden: Comando (HR1) + Número de escena(HR2)
         responder (NO_RESP); 
         EjecutarEscena (Mb.R[MB_OFFSET_COMANDO+1]); 
      break;  
       
      case oAJUSTAR_CONSIGNA_DIA: // Formato orden: Comando (HR1) + idElemento(HR2) + consignaDia(HR3) (grados)
         responder (NO_RESP);
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_CONSIGNA_DIA);
      break;
      
      case oAJUSTAR_MAX_NOCHE: // Formato orden: Comando (HR1) + idElemento(HR2) + Temp. máx en modo noche(HR3) (grados)
         responder (NO_RESP);
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_MAX_NOCHE);
      break;
      
      case oAJUSTAR_MIN_NOCHE: // Formato orden: Comando(HR1) + idElemento(HR2) + Temp. min en modo noche(HR3) (grados)
         responder (NO_RESP);
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_MIN_NOCHE);
      break;
      
      case oAJUSTAR_VELOCIDAD_VENT: // Formato orden: Comando(HR1) + idElemento(HR2) + Velocidad ventilador(HR3)
         responder (NO_RESP);
         actuarTermostato (Mb.R[MB_OFFSET_COMANDO+1], oAJUSTAR_VELOCIDAD_VENT);
      break;
      
      case oALTA_TARJETA_MONEDERO:  // alta de tarjeta 
      {
         responder (NO_RESP);
         byte byte2 = highByte (Mb.R[MB_OFFSET_COMANDO+1]);  //cuarto byte del número de la tarjeta (alto)
         byte byte3 = lowByte (Mb.R[MB_OFFSET_COMANDO+1]);   //tercer byte del número de la tarjeta
         byte byte4 = highByte (Mb.R[MB_OFFSET_COMANDO+2]);  //segundo byte del número de la tarjeta
         byte byte5 = lowByte (Mb.R[MB_OFFSET_COMANDO+2]);   //primer byte del número de la tarjeta (bajo)
          
         unsigned long numero= ((unsigned long)byte2)<<24 | ((unsigned long)byte3)<<16 | ((unsigned long)byte4)<<8 | (unsigned long)byte5;
 
         altaTarjeta(numero);   //dar de alta la tarjeta leída en el maestro TCP
      
         #ifdef debugTCP
         Serial.print ("Tarjeta para dar de alta: ");
         Serial.println (numero,HEX);  
         #endif
      }  
      break;
    
      case oBAJA_TARJETA_MONEDERO:  //baja tarjeta
      {
        responder (NO_RESP);
        byte byte2 = highByte (Mb.R[MB_OFFSET_COMANDO+1]);  //byte ALTO del número de la tarjeta
        byte byte3 = lowByte (Mb.R[MB_OFFSET_COMANDO+1]);   //byte anterior al ALTO del número de la tarjeta
        byte byte4 = highByte (Mb.R[MB_OFFSET_COMANDO+2]);  //byte siguiente al BAJO del número de la tarjeta
        byte byte5 = lowByte (Mb.R[MB_OFFSET_COMANDO+2]);   //byte BAJO del número de la tarjeta
          
        unsigned long numero= ((unsigned long)byte2)<<24 | ((unsigned long)byte3)<<16 | ((unsigned long)byte4)<<8 | (unsigned long)byte5;
  
        bajaTarjeta(numero);   //dar de baja la tarjeta leída en el maestro TCP
      
#ifdef debugTCP
   Serial.print ("Tarjeta para dar de baja: ");
   Serial.println (numero,HEX); 
#endif
      }  
      break;
   
      case oCONSULTAR_SALDO_TARJETA_MONEDERO:  //consultar saldo tarjeta-monedero
      {
        responder (NO_RESP);
        byte byte2 = highByte (Mb.R[MB_OFFSET_COMANDO+1]);  //byte ALTO del número de la tarjeta
        byte byte3 = lowByte (Mb.R[MB_OFFSET_COMANDO+1]);   //byte anterior al ALTO del número de la tarjeta
        byte byte4 = highByte (Mb.R[MB_OFFSET_COMANDO+2]);  //byte siguiente al BAJO del número de la tarjeta
        byte byte5 = lowByte (Mb.R[MB_OFFSET_COMANDO+2]);   //byte BAJO del número de la tarjeta
          
        unsigned long numero= ((unsigned long)byte2)<<24 | ((unsigned long)byte3)<<16 | ((unsigned long)byte4)<<8 | (unsigned long)byte5;
  
        byte saldo=consultarSaldo(numero);
 //ojo        Mb.R[3]= saldo;   //escribo en el registro MB TCP [3] el saldo de la tarjeta RFID   ESTO EN IR CON EL NUMERO TARJETA??
     
#ifdef debugTCP
   Serial.print("Saldo tarjeta= ");
   Serial.println(saldo,DEC);
#endif
      }  
      break; 
      
      case oINCREMENTAR_SALDO_TARJETA_MONEDERO: // Formato orden: Comando(HR1) + Código Tarjeta Monedero_1(HR2) + Código Tarjeta Monedero_2(HR3) + Saldo a incrementar (HR4)
         {
          responder (NO_RESP);
      
          byte byte1 = lowByte (Mb.R[MB_OFFSET_COMANDO+3]);   //saldo a incrementar en la tarjeta
          byte byte2 = highByte (Mb.R[MB_OFFSET_COMANDO+1]);  //byte ALTO del número de la tarjeta
          byte byte3 = lowByte (Mb.R[MB_OFFSET_COMANDO+1]);   //byte anterior al ALTO del número de la tarjeta
          byte byte4 = highByte (Mb.R[MB_OFFSET_COMANDO+2]);  //byte siguiente al BAJO del número de la tarjeta
          byte byte5 = lowByte (Mb.R[MB_OFFSET_COMANDO+2]);   //byte BAJO del número de la tarjeta
          
          unsigned long numero= ((unsigned long)byte2)<<24 | ((unsigned long)byte3)<<16 | ((unsigned long)byte4)<<8 | (unsigned long)byte5;
 
          incrementarSaldo(numero,byte1); //incrementa el saldo de la tarjeta
          byte saldo=consultarSaldo(numero);
//ojo          Mb.R[3]= saldo;  //escribo en el registro MB TCP [3] el saldo de la tarjeta RFID  !!!HACERLO EN LOS IR CON SALDO Y Nº TARJETA
     
          #ifdef debugTCP
          Serial.print("Nuevo saldo=");
          Serial.println(saldo,DEC);
          #endif
          }  
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
       
      comandoRTU[0]= Mb.R[MB_OFFSET_COMANDO+2]; // Valor a enviar 
      
      switch (orden) //Se selecciona la dirección modbus RTU del termostato según el comando
      {
        case oAJUSTAR_CONSIGNA_DIA:   
         anadirOrdenColaRTU (PRESET_SINGLE_REGISTER, id_elemento >> 4, RTU_CONSIGNA_DIA, 1, comandoRTU);
        break;
        
        case oAJUSTAR_MAX_NOCHE:  
         anadirOrdenColaRTU (PRESET_SINGLE_REGISTER, id_elemento >> 4, RTU_MAX_NOCHE, 1, comandoRTU);
        break;
        
        case oAJUSTAR_MIN_NOCHE:  
         anadirOrdenColaRTU (PRESET_SINGLE_REGISTER, id_elemento >> 4, RTU_MIN_NOCHE, 1, comandoRTU);
        break;
        
        case oAJUSTAR_VELOCIDAD_VENT:  
         anadirOrdenColaRTU (PRESET_SINGLE_REGISTER, id_elemento >> 4, RTU_VELOCIDAD_VENT, 1, comandoRTU);
        break;        
      }  
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
    anadirOrdenColaRTU (PRESET_MULTIPLE_REGISTERS, id_elemento >> 4, RTU_HR_COMANDO, RTU_NO_HR, comandoRTU);
    anadirOrdenColaRTU (READ_HOLDING_REGISTERS, id_elemento >> 4, RTU_HR_ADDR, RTU_NO_HR_LECTURA, comandoRTU);
    
#ifdef debugComandos     
      Serial.println ("Configurada orden a elemento digital: ");
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
      anadirOrdenColaRTU (PRESET_MULTIPLE_REGISTERS, id_elemento >> 4, RTU_HR_COMANDO, 1, comandoRTU);
      anadirOrdenColaRTU (READ_HOLDING_REGISTERS, id_elemento >> 4, RTU_HR_ADDR, RTU_NO_HR_LECTURA, comandoRTU);
  

#ifdef debugComandos    
      Serial.println ("Configurada orden a persiana: ");
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
      anadirOrdenColaRTU (PRESET_MULTIPLE_REGISTERS, id_elemento >> 4, RTU_HR_COMANDO, 3, comandoRTU);
      anadirOrdenColaRTU (READ_HOLDING_REGISTERS, id_elemento >> 4, RTU_HR_ADDR, RTU_NO_HR_LECTURA, comandoRTU);

#ifdef debugComandos     
      Serial.println ("Configurada orden a elemento analógico: ");
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
  remoto->no_of_registers = RTU_NO_HR_LECTURA;
  

  switch (tipo){
    case TIPO_DIGITAL:
    case TIPO_LUZ_SIMPLE:
    case TIPO_ENCHUFE:
      remoto->address = RTU_HR_ADDR; 
      break;
    case TIPO_TERMOSTATO:
      remoto->address = RTU_TEMPERATURA;
      break;
      
    case TIPO_MONEDERO:
      remoto->address = RTU_HR_ADDR_MONEDERO;
      remoto->no_of_registers = 2;
    break;
      
  } 
  contadorRTU++; 
  leyendoTipo = tipo;
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
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU / CANT_IR_ESTADO) responder (ERR_DirMbNoValida);
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
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU / CANT_IR_ESTADO) responder (ERR_DirMbNoValida);
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
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU / CANT_IR_ESTADO) responder (ERR_DirMbNoValida);
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
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU / CANT_IR_ESTADO) responder (ERR_DirMbNoValida);
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
// Realiza el alta (inserción en EEPROM) de un equipo esclavo Monedero 
// Actualiza el puntero para una nueva alta
// p: puntero al inicio del array de holding registers
// Responde en los HR:
//   - ERR_IdDuplicado si el idElemento ya se encuentra en EEPROM
//   - ERR_MemoriaInsuficiente si se alcanzó el límite de la zona de elementos
//   - OK si la inserción en EEPROM (alta) se realizó correctamente
//--------------------------------------------------------------------------------------------
void altaMonedero (int* p){
  
  Monedero_t nuevoElemento;
  p= p+MB_OFFSET_COMANDO+1;
  nuevoElemento.idTipo = *(p++);
  nuevoElemento.idElemento = *(p++); 

  
  if (busquedaElemento (nuevoElemento.idElemento) != -1) responder (ERR_IdDuplicado);
  else if ((nuevoElemento.idElemento & 0x000F) != 0) responder (ERR_DirMbNoValida);
  else if ((nuevoElemento.idElemento & 0xFFF0)>>4 > MAX_REMOTOS_RTU / CANT_IR_ESTADO) responder (ERR_DirMbNoValida);
  else if ((punteroEEprom + sizeof (nuevoElemento)) > EP_DIR_MAX_ELEM) responder (ERR_MemoriaInsuficiente);
  else{
   eeprom_write_block((const void*)&nuevoElemento, (void*)punteroEEprom, sizeof (nuevoElemento));
   actualizarPuntero(sizeof (nuevoElemento));    
   responder (OK);
   
#ifdef debugAltas  
   Serial.println ("Alta terminal Monedero: ");
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
   //Mb.R [i] = EEPROM.read (EP_INICIO_TARJETAS_MONEDERO+i-MB_TEST_ALTAS);
  Mb.R [i] = EEPROM.read (EP_INICIO_ELEMENTOS+i-MB_TEST_ALTAS);
}
//--------------------------------------------------------------------------------------------

void responder (int codigo){

   Mb.R[MB_OFFSET_RESPUESTA +idTerminal] = (Mb.R[MB_OFFSET_COMANDO] & 0x0FFF) | (codigo << 8);
#ifdef debugRespuestas
   Serial.print ("Respuesta: "); Serial.print (codigo); Serial.print (" al comando: "); 
   Serial.println (Mb.R[MB_OFFSET_COMANDO] & 0x0FFF, HEX);
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
      break;   
        
      case TIPO_TERMOSTATO:
        *p+=sizeof(termostato); 
      break;
      
      case TIPO_MONEDERO:
        *p+=sizeof(monedero); 
      break;
      
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
      
      EEPROM.write (EP_INICIO_TARJETAS_MONEDERO,(EP_INICIO_TARJETAS_MONEDERO+2) & 0x00FF); 
      EEPROM.write (EP_INICIO_TARJETAS_MONEDERO+1,(EP_INICIO_TARJETAS_MONEDERO+2) >> 8);
      // Se carga el puntero en RAM 
      punteroEEprom = (EEPROM.read (EP_PUNTERO_EEPROM))<<8 | EEPROM.read (EP_PUNTERO_EEPROM+1);
      EEPROM.write (EP_NUM_ELEMENTOS,0); // Cantidad de elementos = 0
      punteroRTU = EP_INICIO_ELEMENTOS;
      inicializarColaRTU();
      responder (OK);
      // faltan ajustes por defecto (IP, máscara, parámetros RTU...) y reiniciar
}
//---------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------
// Busca si la tarjeta leída está de alta en el sistema
//Entrada: número de la tarjeta RFID
//Devuelve:
//      primera dirección en la EEPROM de los datos de la tarjeta leída
//     -1 si no hay tarjetas de alta en el sistema o no encontró la tarjeta en el sistema
//-----------------------------------------------------------------------------------------------
  int buscarTarjeta (unsigned long numTarjeta)  // busca si la tarjeta leída está de alta en el sistema
  {  
    Serial.println ("-----------------------------------------------");
    Serial.print ("numero tarjeta: "); Serial.println (numTarjeta,HEX);
    byte punteroL = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO);
    byte punteroH = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+1);
    int puntero= punteroH <<8 | punteroL;
    Serial.print ("puntero: "); Serial.println (puntero);
    byte ntarjetas=(byte)((puntero-EP_INICIO_TARJETAS_MONEDERO+2)/5);
    Serial.print ("ntarjetas: "); Serial.println (ntarjetas);
    Serial.println ("-----------------------------------------------");
   
    
    if (puntero==EP_INICIO_TARJETAS_MONEDERO+2)   // no hay tarjetas de alta en el sistema
      {
        return -1;  // No hay tarjetas de alta en el sistema
      }
    else
      {
        for (int i=0; i<ntarjetas; i++)
          {
                         
            if (
               (EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+2+5*i) == (numTarjeta & 0x000000FF)) && 
               (EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+3+5*i) == (numTarjeta & 0x0000FF00)>>8) &&
               (EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+4+5*i) == (numTarjeta & 0x00FF0000)>>16) &&
               (EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+5+5*i) == (numTarjeta & 0xFF000000)>>24)
               )
               
               
               {
                                                
                 //Serial.print("Primera direccion en la EEPROM de los datos de la tarjeta leida: ");
                 //Serial.println(EP_INICIO_TARJETAS_MONEDERO+2+5*i, DEC); 
                 return EP_INICIO_TARJETAS_MONEDERO+2+5*i;//primera direcciÃ³n en la EEPROM de los datos de la tarjeta leÃ­da:
                                              //4 bytes para el nÃºmero de tarjeta y 1 byte para el saldo de la tarjeta
                             
               }                               
                         
          }
          //Serial.println("No se encuentra la tarjeta buscada en la EEPROM");
          return -1;   // no encontrÃ³ la tarjeta buscada en la EEPROM
      }  
      
    
    
  }
  
  //---------------------------------------------------------------------------------------------
  //Da de alta tarjetas RFID guardándolas en la EEPROM
  //Entrada: número de la tarjeta RFID
  //Devuelve:
  //      1 si la tarjeta RFID se ha dado de alta correctamente en el sistema
  //     -1 si la tarjeta RFID ya estaba de alta en el sistema
  //---------------------------------------------------------------------------------------------
  int altaTarjeta(unsigned long numTarjeta)
  {
    
    if (buscarTarjeta(numTarjeta)== -1) // Tarjeta no dada de alta
      {
        byte punteroL = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO);
        byte punteroH = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+1);
        int puntero= punteroH <<8 | punteroL;
        
        EEPROM.write(puntero,numTarjeta & 0x000000FF);              //guardar los 8 bits menos significativos del nÂº de la tarjeta (byte1)
        EEPROM.write(puntero+1,(numTarjeta & 0x0000FF00)>>8);        //guardar byte2 del nÂº de la tarjeta
        EEPROM.write(puntero+2, (numTarjeta & 0x00FF0000)>>16);     //guardar byte3 del nÂº de la tarjeta
        EEPROM.write(puntero+3, (numTarjeta & 0xFF000000)>>24);     //guardar los 8 bits mÃ¡s significativos del nÂº de la tarjeta (byte4)
        EEPROM.write(puntero+4,0);                                  //saldo cero para la tarjeta dada de alta ???
                      
        
        puntero+= 5; // 
        
         EEPROM.write(EP_INICIO_TARJETAS_MONEDERO, puntero & 0x00FF); 
         EEPROM.write(EP_INICIO_TARJETAS_MONEDERO+1, (puntero & 0xFF00) >> 8);
         return 1; //la tarjeta se ha dado de alta en el sistema       
      }
    else return -1;  //la tarjeta ya existe en la EEPROM. Ya estaba de alta.  
  }
  
  //---------------------------------------------------------------------------------------------
  //Para dar de baja una tarjeta RFID en el sistema
  //Entrada: número de la tarjeta RFID
  //Devuelve:
  //      1 si la tarjeta se ha dado de baja en el sistema
  //     -1 si no se ha encontrado la tarjeta que se quiere dar de baja
  //---------------------------------------------------------------------------------------------
  
  int bajaTarjeta(unsigned long numTarjeta)
  {
    
    if (buscarTarjeta(numTarjeta)== -1) return -1; // tarjeta no encontrada
    else
      {     
       //muevo la Ãºltima tarjeta para el hueco que deja la que se da de baja 
       int punteroBaja=buscarTarjeta(numTarjeta);    //direcciÃ³n de la tarjeta que se da de baja
           
       //muevo el puntero 5 posiciones hacia arriba para leer la Ãºltima tarjeta que estÃ¡ en la EEPROM
       byte punteroL = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO);
       byte punteroH = EEPROM.read(EP_INICIO_TARJETAS_MONEDERO+1);
       int puntero= punteroH <<8 | punteroL;
       puntero=puntero-5;
       EEPROM.write(EP_INICIO_TARJETAS_MONEDERO,puntero & 0x00FF);   //guardar el valor del puntero en EP_INICIO_TARJETAS_MONEDERO y EP_INICIO_TARJETAS_MONEDERO+1
       EEPROM.write(EP_INICIO_TARJETAS_MONEDERO+1,(puntero & 0xFF00) >> 8);   
              
       byte valor=EEPROM.read(puntero);          //leo la Ãºltima tarjeta y la coloco en el lugar de la que se diÃ³ de baja
       if (EEPROM.read(punteroBaja)!=valor)
	   {
		EEPROM.write(punteroBaja,valor);
	   }
       valor=EEPROM.read(puntero+1);
       if (EEPROM.read(punteroBaja+1)!=valor)
	   {
		EEPROM.write(punteroBaja+1,valor);
	   }
       valor=EEPROM.read(puntero+2);
       if (EEPROM.read(punteroBaja+2)!=valor)
	   {
		EEPROM.write(punteroBaja+2,valor);
	   }
       valor=EEPROM.read(puntero+3);
       if (EEPROM.read(punteroBaja+3)!=valor)
	   {
		EEPROM.write(punteroBaja+3,valor);
	   }
       valor=EEPROM.read(puntero+4);
       if (EEPROM.read(punteroBaja+4)!=valor)
	   {
		EEPROM.write(punteroBaja+4,valor);
	   }
       
       return 1;  //la tarjeta se ha dado de baja en el sistema     
      }    
  }
  
  
  //----------------------------------------------------------------------------------------------
  //Para consultar el saldo de una tarjeta RFID
  //Entrada: número de la tarjeta RFID
  //Devuelve:
  //        -1 si no se ha encontrado la tarjeta en el sistema
  //        saldo de la tarjeta
  //----------------------------------------------------------------------------------------------
  byte consultarSaldo(unsigned long numTarjeta)
  {
    if (buscarTarjeta(numTarjeta)== -1) return -1; //no se ha encontrado la tarjeta en la EEPROM: no estaba de alta
                     //devuelve un valor de 255
    else return EEPROM.read(buscarTarjeta(numTarjeta)+4);  //lee el valor del saldo de la tarjeta     
  }  
  
 //----------------------------------------------------------------------------------------------
 //Para incrementar el saldo de una tarjeta RFID
 //Entrada: número de la tarjeta RFID y valor del incremento de saldo que se desea realizar
 //Devuelve:
 //      -1 si no se ha encontrado la tarjeta en el sistema
 //       0 si se pretende un incremento de saldo negativo o si saldo+incrSaldo>254
 //----------------------------------------------------------------------------------------------
  byte incrementarSaldo(unsigned long numTarjeta, byte incrSaldo)
  {
    if (buscarTarjeta(numTarjeta)== -1) return -1; //no se ha encontrado la tarjeta en la EEPROM: no estaba de alta
    else
      { 
       byte valor=EEPROM.read(buscarTarjeta(numTarjeta)+4);  //lee el valor del saldo de la tarjeta
       // comprobar que es un valor entero
       if ((incrSaldo)>0 & (valor+incrSaldo<255)){
         EEPROM.write(buscarTarjeta(numTarjeta)+4,valor+incrSaldo);  //guarda el nuevo valor de saldo de la tarjeta
         return 1;
       }
       else return 0; //mejor poner return distinto de -1 para ver por quÃ© sale de la funciÃ³n. AquÃ­ es por incrSaldo negativo o porque valor+incrSaldo>254
      }  
  }   
  //----------------------------------------------------------------------------------------------
  //Para decreemntar el saldo de una tarjeta RFID
  //Entrada: número de la tarjeta RFID y valor de la cantidad que se quiere decrementar
  //Devuelve:
  //        -1 si no se ha encontrado la tarjeta en el sistema
  //         0 si no se puede decrementar el saldo porque se quedaría en números rojos
  //----------------------------------------------------------------------------------------------
  byte decrementarSaldo(unsigned long numTarjeta, byte decrSaldo)
  {
    if (buscarTarjeta(numTarjeta)== -1)
      {
       return -1; //no se ha encontrado la tarjeta en la EEPROM: no estaba de alta
                    
      }
    else
      { 
       byte valor=EEPROM.read(buscarTarjeta(numTarjeta)+4);  //lee el valor del saldo de la tarjeta
       if ((decrSaldo)>0 & (valor-decrSaldo>=0))  //no pongo saldos negativos con el formato byte o sÃ­ los ponemos????
        {
          EEPROM.write(buscarTarjeta(numTarjeta)+4,valor-decrSaldo);  //guarda el nuevo valor de saldo de la tarjeta
        }
       else
        {
          return 0; // no puedo decrementar el saldo
        } 
      }  
  }  
 //--------------------------------------------------------------------------------------------------------------

void procesarMonedero (){
    
  byte saldo;
  
  decrementarSaldo ((long)(tarjetaRFID[0])<<16 | tarjetaRFID[1], 1); //de momento se decrementa 1 unidad
  saldo = consultarSaldo((long)(tarjetaRFID[0])<<16 | tarjetaRFID[1]);
  
  if (saldo>0 && saldo<255) comandoRTU[0]= RTU_OK_MONEDERO;   
  else comandoRTU[0]= RTU_NACK_MONEDERO;  
  comandoRTU[1]= saldo; 
  anadirOrdenColaRTU (PRESET_MULTIPLE_REGISTERS, dirMB_Monedero, RTU_HR_COMANDO, 2, comandoRTU);
  tarjetaRFID[0]=0; tarjetaRFID[1]=0; //borramos tarjeta leída

#ifdef debugComandos     
      Serial.println ("Configurada orden a monedero: ");
      Serial.println ("------------------------------------------");
#endif   
   
}
//-----------------------------------------------------------------------------------------------
void inicializarColaRTU (){
  for (byte i=0; i<TAMANO_COLA_RTU; i++)
    for (byte j=0; i<4+RTU_NO_HR; j++) colaRTU[i][j]= 0;
}
//------------------------------------------------------------------------------------------------
  
void eliminarOrdenColaRTU (){
  for (byte i=1; i<TAMANO_COLA_RTU; i++)
    for (byte j=0; j<4+RTU_NO_HR; j++) colaRTU [i-1][j]= colaRTU[i][j];
  for (byte k=0; k<4+RTU_NO_HR; k++) colaRTU [TAMANO_COLA_RTU-1][k] = 0; //borramos último elemento  
}
//-------------------------------------------------------------------------------------------------

void anadirOrdenColaRTU (byte _funcion, byte _id, byte _dirHR, byte _numRegs, unsigned int* _datos) {
  
    byte i;
    unsigned int* aux_datos;
    aux_datos = _datos; 
    for (i=0; i< TAMANO_COLA_RTU; i++)
      if (colaRTU[i][0] == 0) break;
      
    if (i >= TAMANO_COLA_RTU-1){
      responder (ERR_desbordamiento_cola_rtu);
      return;
    }  
  
    colaRTU [i][0] = _funcion; // función Modbus (3,6 o 16);
    colaRTU [i][1] = _id;      // dirección modbus del remoto
    colaRTU [i][2] = _dirHR;   // dirección de los holding registers a leer o escribir
    colaRTU [i][3] = _numRegs; // Número de registros a leer o escribir
    
    
    if (_funcion ==6 || _funcion == 16){
       for (byte j=0; j < _numRegs; j++) {  //
          Serial.print (4+j); Serial.print (": "); Serial.println (*_datos);
          colaRTU[i][4+j] = *_datos;
          _datos++;
       }    
    }
    
#ifdef debugRTU    
  Serial.println ("--------------------------------------------------");
  Serial.print ("insertado elemento en cola en posicion: "); Serial.println (i);  
  Serial.print ("cola_funcion: "); Serial.println (colaRTU[i][0]);
  Serial.print ("cola_id: "); Serial.println (colaRTU[i][1]);
  Serial.print ("cola_dirHR: "); Serial.println (colaRTU[i][2]);
  Serial.print ("cola_numRegs: "); Serial.println (colaRTU[i][3]);  
  for (byte j=0; j < _numRegs; j++) {
    Serial.print ("comando: "); Serial.print (j); Serial.print (" : "); Serial.println (*aux_datos);
    aux_datos++;
  }
  Serial.println ("--------------------------------------------------");
#endif    
}
//------------------------------------------------------------------------------------------------

byte tamanoColaRTU (){
    byte i;
    for (i=0; i< TAMANO_COLA_RTU; i++)
      if (colaRTU[i][0] == 0) break;
    return i;
}
//------------------------------------------------------------------------------------------------
    
      
