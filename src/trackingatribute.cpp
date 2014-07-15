#include "trackingatribute.h"
#include <math.h>

trackingAtribute::trackingAtribute(double valorActual){
    //Se ejecuta solo 1 vez y al comienzo, por lo que los valores pasados no existen
    //y se dejan en 1 para evitar divisiones por 0
    A = 2.0; lambda = 6;
    a_exp = 1; a_est = 1;
    Sa = 1;
    Var_a = 1; Va = 1;
    S_Va = 1;
    //Valores anteriores tambien se inicializan en 1 para evitar las divisiones por 0
    prev_a0 = 1;
    prevA_est = 1;
    prevSa = 1;
    prevVar_a = 1;
    prevVa = 1;
    prevS_Va = 1;
    mean_a = (a_exp+a_est)/2;
    prevMean_a = 1;
    a0 = valorActual;
    Va0 = (a0-prev_a0)/1;
}

void trackingAtribute::refreshAtributes(double valorActual){
    //Los valores anteriormente seteados para las variables, se transforman en valores previos
    prev_a0 = a0;
    prevA_est = a_est;
    prevSa = Sa;
    prevVar_a = Var_a;
    prevVa = Va;
    prevS_Va = S_Va;
    prevMean_a = mean_a;

    //Se empiezan a calcular los nuevos valores con el parametro valorActual como a0
    a0 = valorActual;
    Va0 = (a0-prev_a0)/1;

    //Las siguientes expresiones corresponden al modelo de Dinamica de Atributos
    Sa = 1 + exp(-lambda)*prevSa;
    a_est = (a0+exp(-lambda)*prevA_est*Sa)/prevSa;
    S_Va = 1 + exp(-lambda)*prevS_Va;
    Va = Va0 + exp(-lambda)*prevVa*prevS_Va;
    a_exp = (prevMean_a + Va);
    Var_a = sqrt(exp(-lambda)*prevSa*(pow(prevVar_a,2) + pow(a0 - prevMean_a,2)/Sa)/Sa);
}

double trackingAtribute::calculateProbability(){
    //Se calcula la probabilidad ajustando el valor mediante la constante A de acuity
    return (A/Var_a);
}
