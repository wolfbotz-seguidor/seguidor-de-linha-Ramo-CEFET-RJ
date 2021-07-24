int PID(int error)/*Algoritmo de controle PID usando os sensores frontais*/
{
    static unsigned int Kp = 2, Kd = 0, Ki = 0;
    static unsigned int prescale = 2048; //prescale na pot?ncia de 2: 2^n
    static int integral = 0, erroAnterior = 0;
    int p = 0, i = 0, d = 0, Turn = 0;
   
    p = (error * Kp); // Proporcao

    integral += error; // Integral
    i = (Ki * integral);

    d = (Kd * (error - erroAnterior)); // Derivada
    erroAnterior = error;

    Turn = (p + i + d) / prescale;

    return Turn;
}