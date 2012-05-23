
//Ajustes modbus RTU
#define BAUD 19200
#define TIMEOUT 1000
#define POLLING 5 
#define RETRY_COUNT 1 
#define TX_ENABLE_PIN 0
#define RTU_MAX_NO_HR 1 // Cantidad de HR máxima en los remotos

#define RTU_HR_ADDR 2

#define RTU_HR_COMANDO 0 // Dirección Modbus en la que los remotos aceptan órdenes
#define RTU_NO_HR 3 // Número de holding registers en el remoto (tamaño de la orden)

#define MAX_REMOTOS_RTU 116 //Número máximo de remotos RTU

enum
{
  REMOTO,
  NUM_REMOTOS
};

Packet remotos[NUM_REMOTOS];
packetPointer remoto = &remotos[REMOTO];
unsigned int regs[RTU_MAX_NO_HR]; //Holding registers en RTU
unsigned int comandoRTU [RTU_NO_HR]; // Comando RTU a enviar a los remotos
unsigned int erroresRTU = 0; // Número de errores en las comunicaciones RTU
unsigned int okRTU =0 ; // Número de sondeos correctos RTU
boolean lecturaRTU = true; // sondeo cíclico RTU habilitado
byte contadorRTU = 0; // Número de remotos que responden correctamente (RTU)
boolean MbRTU_Leido [117]; // = true => La dirección modbus ya fue sondeada este ciclo (RTU)
unsigned int resultadoRTU= 0; // 0-En curso, 1-Finalizada OK, 2-Finalizada con error
boolean cambio = true; // Cambio a sondeo de nuevo remoto RTU

