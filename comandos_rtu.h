


//WK0600
#define TIPO_TERMOSTATO 7
#define RTU_TEMPERATURA 101 //decimas de grado
#define RTU_CONSIGNA_DIA 135 //grados
#define RTU_MAX_NOCHE 120 //grados
#define RTU_MIN_NOCHE 119 //grados
#define RTU_VELOCIDAD_VENT 137 // 0= apagado, 4=auto
#define RTU_MODO_DIA 184 //leer registro y poner bit 0 a 1
#define RTU_MODO_NOCHE 184 //leer registro y poner bit 0 a 0
#define RTU_TERM_DO 108 // bits 0, 1, 2 = velocidades fancoil
#define RTU_VALVULA_FRIO 102 //0..1000
#define RTU_VALVULA_CALOR 103 //0..1000

//Modbus RTU
#define RTU_ACTIVACION_REMOTO 0x25 // Orden de activación en el remoto
#define RTU_DESACTIVACION_REMOTO 0x26 // Orden de desactivación en el remoto
#define RTU_REGULACION_REMOTO 0x27 // Orden de regulación en el remoto
#define RTU_ACTIVACION_PERSIANA 0x28
#define RTU_DESACTIVACION_PERSIANA 0x29
#define RTU_DETENER_PERSIANA 0x2A
