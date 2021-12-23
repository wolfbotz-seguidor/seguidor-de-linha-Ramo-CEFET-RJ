char PID(char error)/*Algoritmo de controle PID usando os sensores frontais*/
{
<<<<<<< Updated upstream
    static unsigned int Kp = 2, Kd = 0, Ki = 0;
    static unsigned int prescale = 4; //prescale na potência de 2: 2^n //4 = 16
=======
    static int Kp = 18, Kd = 40, Ki = 0;
    static int prescale = 6; //prescale na potência de 2: 2^n //4 = 16
>>>>>>> Stashed changes
    static int integral = 0;
    static char erroAnterior = 0;
    int p = 0, i = 0, d = 0;
    char Turn = 0;
   
    p = (error * Kp); // Proporcao

    integral += error; // Integral
    i = (Ki * integral);

    d = (Kd * (error - erroAnterior)); // Derivada
    erroAnterior = error;

    Turn = (p + i + d) >> prescale;

    return Turn;
}