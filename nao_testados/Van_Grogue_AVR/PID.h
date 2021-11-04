int PID(int error)/*Algoritmo de controle PID usando os sensores frontais*/
{
    static int Kp = 30, Kd = 390, Ki = 0;
    static int prescale = 6; //prescale na potência de 2: 2^n //4 = 16
    static int integral = 0;
    static int erroAnterior = 0;
    static int p = 0, i = 0, d = 0;
    int PID = 0;
   
    if(!error)  integral = 0;
    
    p = (error * Kp); // Proporcao

    integral += error; // Integral
    
    if(integral > 1023)         integral =  1023;
    else if(integral < -1023)   integral = -1023;
    
    i = (Ki * integral);

    d = (Kd * (error - erroAnterior)); // Derivada
    erroAnterior = error;

    PID = (p + i + d) >> prescale;
    
    
    return PID;
}