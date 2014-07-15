#ifndef TRACKINGATRIBUTE_H
#define TRACKINGATRIBUTE_H

class trackingAtribute
{
public:
    trackingAtribute(double);

    //a0: valor actual del atributo
    //prev_a0: valor anterior (del frame anterior)
    //Va0 = a0 - prev_a0; es la velocidad instantanea, frame a frame
    double a0,prev_a0,Va0;

    // Corresponden a las variables de Dinamica de Atributos.
    // a_exp y a_est. Se usan para  calcular los otros parametros.
    double a_exp, a_est;
    // mean_a es el promedio entre a_exp y a_est, es decir,
    // mean_a=(a_exp - a_est)/2.
    double mean_a;


    // La variable mas importante es Var_a, que contiene la varianza, esta se usa para calcular
    // la probabilidad del movil.
    double Sa, Var_a, Va, S_Va;


            // Toda variable con el prefijo "prev" contiene el valor de la variable, pero del frame
    // anterior.
    double prevA_exp, prevA_est, prevMean_a, prevSa, prevVar_a, prevVa, prevS_Va;


    // refreshAtributes(double), toma por input a0, el valor actual del atributo
    // re-calcula las variables y actualiza el valor de todas las variables.
    void refreshAtributes(double);

    // calculateProbability(), retorna el valor de probabilidad del atributo,
    // para un cierto a0 que puede ser el valor actual del atributo o el valor
    // de alguna correspondencia
    double calculateProbability();

private:
    // lambda es tambien conocido como el factor de enfriamiento, pondera
    // en cierto modo, la influencia de los frames anteriores en el calculo
    // de los parametros.
    double lambda;

    // factor de acuity (agudeza), es dividido por la varianza para obtener
    // la probabilidad. Esta constante ajusta el valor de 1/Var_a para el
    // orden de magnitud de las mediciones
    double A;


};

#endif // TRACKINGATRIBUTE_H
