

double read_phase_a_cur(){
  ads1015.setGain(GAIN_TWO);
  double temp = ads1015.computeVolts(ads1015.readADC_SingleEnded(0));
  // current is  ((V-offset)/amp_gain)/R = I
  double cur = -((temp - 1.65)/50)/0.002;
  return cur;
}

double read_phase_b_cur(){
  ads1015.setGain(GAIN_TWO);
  double temp = ads1015.computeVolts(ads1015.readADC_SingleEnded(1));
    // current is  (V/amp_gain)/R = I
  double cur = -((temp - 1.65)/50)/0.002;
  return cur;
}

double read_in_cur(double ref){
  ads1015_B.setGain(GAIN_TWO);
  double temp = 0;
  for (int i = 0; i < avgCountCS; i++) {
    temp = temp + ads1015_B.computeVolts(ads1015_B.readADC_SingleEnded(1));
  }
    // current is  (V/amp_gain)/R = I
  double cur = (((temp/avgCountCS) - ref)/50)/0.0005;
  return cur;
}

double read_out_cur(double ref){
  ads1015.setGain(GAIN_TWO);
  double temp = ads1015.computeVolts(ads1015.readADC_SingleEnded(2));
  // current is  (V/amp_gain)/R = I
  // Serial.print((String)"in cur: " + temp)
  double cur = ((temp - ref)/50)/0.0005;
  return cur;
}

double read_in_volt(){
  ads1015_B.setGain(GAIN_TWO);
  double temp = 0;
  for (int i = 0; i < avgCountVS; i++) {
    temp = temp + ads1015_B.computeVolts(ads1015_B.readADC_SingleEnded(0));
  }
  // volts is v divider: (4.125/104.125)* v_meas
  double volts = (temp/avgCountVS)/(4.125/104.125);
  return volts;

}

double read_out_volt(){
  ads1015.setGain(GAIN_TWO);
  double temp = ads1015.computeVolts(ads1015.readADC_SingleEnded(3));
  double volts = temp/(4.125/104.125);
  return volts;
}

double cal_in_cur(double cur){
  return cur*0.701 + 0.00624;
  return cur;
}

double cal_out_cur(double cur){
  return cur*-0.904 -0.00609;
  return cur;
}

void read_sensors(){
  double ref = ads1015_B.computeVolts(ads1015_B.readADC_SingleEnded(2));
  Serial.print((String)"ref: " + ref );
  vin = read_in_volt();
  vout = read_out_volt();
  cin = read_in_cur(ref);
  // cin = cin - cin_tare;
  cin = cal_in_cur(cin);
  cout = read_out_cur(ref);
  // cout = cout - cout_tare;
  cout = cal_out_cur(cout);
  c_a = read_phase_a_cur();
  c_b = read_phase_b_cur();
  powerInput = vin * cin;
  powerOutput = vout * cout;
  outputDeviation = (vout / 20) * 100.000;
}



