
// Tipos de elementos domóticos

#define TIPO_DIGITAL 1
#define TIPO_ANALOGICO 2
#define TIPO_PROG_HORARIO 3
#define TIPO_ENCHUFE 4
#define TIPO_LUZ_SIMPLE 5
#define TIPO_LUZ_REGULADA 6
#define TIPO_TERMOSTATO 7
#define TIPO_PERSIANA 8
#define TIPO_MONEDERO 9


#define LED_AMARILLO 60
#define LED_ROJO 61


//Mapa modbus TCP
#define MB_OFFSET_COMANDO 0
#define MB_OFFSET_FECHA_HORA 10
#define MB_TEST_ALTAS 20
#define MB_OFFSET_RESPUESTA 16
#define IR_OFFSET_COMOK 117


//Mapa EEPROM
#define EP_PUNTERO_EEPROM 31
#define EP_NUM_ELEMENTOS 30
#define EP_DIR_MAX_ELEM  3589
#define DIR_MAX_EEPROM 4095
#define EP_INICIO_TARJETAS_MONEDERO 3590  //para 100 tarjetas-monedero como máximo


#define EP_INICIO_ELEMENTOS 40
