// costandi controllo PID
#define KP_rel ?
#define KI_rel ?
#define KD_rel ?

#define KP_ass ?
#define KI_ass ?
#define KD_ass ?

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  //si legge la velocità dell'encoder Relativo e la posizione dell'encoder Assoluto
  velREL = ;

  //velocità di output Encoder Relativo
  outputREL = (setPoint_REL - velREL) * KP_REL;   // contributo proporzionale
  outputREL += (sumErr * KI_REL * DT / 1000);         // contributo integrativo
  outputREL +=  -(1000 * KD_REL * (encoderAngle - oldEncoderAngle) / DT ); //contributo derivativo


  sommaErr_REL += setPoint_REL - velREL;

  //scrittura della nuova velocità
  analogWrite(PIN_PWM, outputREL);


  //Lettura della posizione dell'encoder Assoluto
  posizioneASS = ;

  //velocità di output Encoder Assoluto
  outputASS = (setPoint_ASS - posizioneASS) * KP_ass;   // contributo proporzionale
  outputASS += (sumErr * KI_ASS * DT / 1000);         // contributo integrativo
  outputASS +=  -(1000 * KD_ASS * (encoderAngle - oldEncoderAngle) / DT ); //contributo derivativo


  sommaErr_ASS += setPoint_ASS - posizioneASS;

  //scrittura della nuova velocità
  analogWrite(PIN_PWM, outputASS);

  
}
