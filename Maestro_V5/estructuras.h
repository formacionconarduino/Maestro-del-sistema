
//--------------------------------------------------------------------
//Elemento digital

typedef struct {
  byte idTipo;     // byte: 1-255
  unsigned int idElemento; // 12 bits de mayor peso = dirección MB; 4 bits de menor peso = num. salida
  unsigned int escenas; // 16 escenas; bit=0 = elemento no pertenece a la escena; bit=1 elemento pertenece a la escena
  unsigned int valoresEscenas; // valor del elemento al ejecutar cada escena
} Elem_dig_t;
//---------------------------------------------------------------------

//Elemento analógico

typedef struct {
  byte idTipo;
  unsigned int idElemento;
  unsigned int escenas;
  byte valoresEscenas [16];
} Elem_ana_t;

//----------------------------------------------------------------------
//Programa horario

typedef struct {
  byte idTipo;
  byte idPrograma;
  unsigned int idElemento;
  byte activacion; //elemento/escena/grupo/habilitado/ejecutado
  byte hora; //hora y minuto. resolución 10 min.
  byte dia_semana;// 7 bits + 1 bit (semanal/ una vez)
  byte valor;
} Phorario_t;

typedef struct {
  byte idTipo;
  byte idPrograma;
  boolean esElemento;
  boolean esEscena;
  boolean esGrupo;
  boolean habilitado;
  boolean ejecutado;
  boolean semanal;
  byte dirModbus;
  byte canal;
  byte valor;
} Phorario_ext_t;

//----------------------------------------------------------------------
typedef struct {
  byte idTipo;
  unsigned int idElemento;
} Termostato_t;
//----------------------------------------------------------------------

typedef struct {
  byte idTipo;
  unsigned int idElemento;
} Persiana_t;
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//Monedero electrónico
typedef struct {
  byte idTipo;
  unsigned int idElemento;
} Monedero_t;
//----------------------------------------------------------------------

