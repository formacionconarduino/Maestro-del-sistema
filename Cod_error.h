
#define OK 0
#define NO_RESP 15
#define ERR_NoRTC 8                  // RTC no instalado
#define ERR_Fecha 1                  // Formato fecha no válido
#define ERR_Hora 2                   // Formato hora no válido
#define ERR_IdNoEncontrado 3         // Identificador no se encuentra
#define ERR_TipoNoValido 4           // Tipo de elemento no válido o no soportado
#define ERR_ComTermostato 5          // Error de comunicación con el termostato
#define ERR_Tipo 6                   // El elemento (id_elemento) no es de ese tipo
#define ERR_ComDigital 7             // Error de comunicación con el elemento digital
#define ERR_MemoriaInsuficiente 9    // No se inserta el elemento porque no existe memoria EEPROM disponible
#define ERR_IdDuplicado 10           // No se inserta el elemento porque el idElemento ya se encuentra en la tabla de elementos
#define ERR_ComAnalogico 11          // Error de comunicación con el elemento analógico
#define ERR_ComPersiana 13
#define ERR_Escena_no_valida 14
#define ERR_valor_fuera_de_rango 12  // Valor de regulación analógica fuera de rango
#define ERR_DirMbNoValida 30
