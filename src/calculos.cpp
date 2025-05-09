#include "calculos.h"

// Adafruit_ADS1115 ads;

float Fc = 10.0;                  // Frecuencia de corte en Hz (ajustable)
float Fs = 500.0;                 // Frecuencia de muestreo en Hz (ajustable)
float a1, a2, b0, b1, b2;         // Coeficientes del filtro Butterworth
float x[3] = {0.0, 0.0, 0.0};     // Últimos valores de entrada
float y[3] = {0.0, 0.0, 0.0};     // Últimos valores de salida

/************************************************************************************************************
 * @fn      sliding_moving_average
 * @brief   
 * @param   
 * @return  
 * TODO: - 
 */
void sliding_moving_average(float* input, uint16_t N, uint8_t M, float* output) {
  float accumulator = 0.0;
  uint8_t aux = (uint8_t)((M - 1) / 2);

  // Para las primeras aux+1 muestras
  for ( uint8_t i = 0; i < M; i++ ) {
    accumulator += input[i];
    if ( i >= aux ) {
      output[i - aux] = accumulator / (i + 1);
    }
  }

  // Para el resto de las muestras (ventanas deslizantes completas)
  for ( uint16_t i = M - aux; i < N - aux; i++ ) {
    accumulator += input[i + aux] - input[i - aux - 1];  // Actualiza el acumulador
    output[i] = accumulator / M;  // Calcula el promedio
  }

  // Para los últimos valores
  for ( uint16_t i = N - aux; i < N; i++ ) {
    accumulator -= input[i - aux - 1];  // restar el valor sobrante
    output[i] = accumulator / (N - i + aux);  // Promedio con los valores restantes
  }
}

/************************************************************************************************************
 * @fn      init_butterworth
 * @brief   
 * @param   
 * @return  
 */
void init_butterworth(void) {
  float omega = 2 * PI * Fc / Fs;
  float sn = sin(omega);
  float cs = cos(omega);
  float alpha = sn / sqrt(2);   // Q factor para Butterworth de 2do orden
  
  // Coeficientes de Butterworth
  b0 = (1 - cs) / 2;
  b1 = 1 - cs;
  b2 = (1 - cs) / 2;
  a1 = -2 * cs;
  a2 = 1 - alpha;

  float norm = 1 + a1 + a2;     // Normalización de coeficientes

  b0 /= norm;
  b1 /= norm;
  b2 /= norm;
  a1 /= norm;
  a2 /= norm;
}

/************************************************************************************************************
 * @fn      apply_butterworth
 * @brief   
 * @param   
 * @return  
 */
float* apply_butterworth(float *input, uint16_t Elementos) {
  // Crear un nuevo arreglo para almacenar la salida filtrada
  float *filtered_output = (float*)malloc(Elementos * sizeof(float));
  if (filtered_output == NULL) {
    Serial.println("Error: No se pudo asignar memoria para la salida del filtro.");
    return NULL;
  }

  // Reiniciar las variables del filtro
  for (int i = 0; i < 3; i++) {
    x[i] = 0.0;
    y[i] = 0.0;
  }

  // Aplicar el filtro a cada elemento
  for (uint8_t i = 0; i < Elementos; i++) {
    x[2] = x[1];
    x[1] = x[0];
    x[0] = input[i];

    y[2] = y[1];
    y[1] = y[0];
    y[0] = b0 * x[0] + b1 * x[1] + b2 * x[2] - a1 * y[1] - a2 * y[2];

    filtered_output[i] = y[0];
  }

  return filtered_output;
}
// float apply_butterworth(float new_input) {
//   x[2] = x[1];          // Desplazar las muestras anteriores
//   x[1] = x[0];
//   x[0] = new_input;

//   y[2] = y[1];
//   y[1] = y[0];
//   y[0] = b0 * x[0] + b1 * x[1] + b2 * x[2] - a1 * y[1] - a2 * y[2];  // Filtro de segundo orden
  
//   return y[0];
// }

/************************************************************************************************************
 * @fn      obtain_Vbias
 * @brief   Se obtiene la curva I-V inversa del SiPM, se aplica la inversa de la derivada del logaritmo neperiano de la curva. El pico de
 *          esta es el Voltaje de ruptura, sumandole el over voltage tenemos Vbias.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @return  ---todo
 */
float obtain_Vbd(float *inverseCurrent_I, float *inverseVoltage, uint16_t Elementos) {
  float logarithmicCurrent[Elementos];
  float derivative[Elementos];
  float inverseDerivative[Elementos];
  float BreakdownVoltage = inverseVoltage[0];
  int indexPeak = -1;

  for ( uint8_t i = 0; i < Elementos; i++ ) {            // Logaritmo natural de 
    if ( inverseCurrent_I[i] > 0.0f ) {
      logarithmicCurrent[i] = logf(inverseCurrent_I[i]);
    } else {
      logarithmicCurrent[i] = NAN;
    }
  }

  derivative[0] = (logarithmicCurrent[1] - logarithmicCurrent[0]) / (inverseVoltage[1] - inverseVoltage[0]);
  derivative[Elementos - 1] = (logarithmicCurrent[Elementos - 1] - logarithmicCurrent[Elementos - 2]) /
                                  (inverseVoltage[Elementos - 1] - inverseVoltage[Elementos - 2]);

  for ( uint8_t i = 1; i < Elementos - 1; i++ ) {
    float deltaVoltage = inverseVoltage[i + 1] - inverseVoltage[i - 1];
    if (deltaVoltage != 0.0f && isfinite(logarithmicCurrent[i + 1]) && isfinite(logarithmicCurrent[i - 1])) {
      derivative[i] = (logarithmicCurrent[i + 1] - logarithmicCurrent[i - 1]) / deltaVoltage;
    } else {
      derivative[i] = NAN; // Asignar NAN si no es válida
    }
    if (isfinite(derivative[i]) && derivative[i] > 0.0f) {
      inverseDerivative[i] = 1.0f / derivative[i];
    } else {
      inverseDerivative[i] = NAN; // Manejar casos no válidos
    }
  }

  float maxInverseDerivative = -FLT_MAX;
  for (uint8_t i = 1; i < Elementos - 1; i++) {
    if (isfinite(inverseDerivative[i]) && inverseDerivative[i] > maxInverseDerivative) {
      maxInverseDerivative = inverseDerivative[i];
      indexPeak = i;
    }
  }

  if (indexPeak != -1) {
    BreakdownVoltage = inverseVoltage[indexPeak];
  } else {
    BreakdownVoltage = NAN; // Retornar NAN si no se encontró un pico válido
  }

  return inverseVoltage[indexPeak];
}

/************************************************************************************************************
 * @fn      Vbd_teorical
 * @brief   Voltaje de ruptura teorica de acuerdo a la temperatura
 * @param   Temperature
 * @return  valor del voltaje estimado
 */
float Vbd_teorical(float Temperature) {   // Celcius
  return 23.9985 + 0.0215 * Temperature;  // Curva lineal
}

/************************************************************************************************************
 * @fn      V_to_command
 * @brief   Conversión de voltaje a commando bin para el MAX
 * @param   Voltaje: Es el voltaje objetivo en la salida del MAX
 * @return  comando en binario
 */
